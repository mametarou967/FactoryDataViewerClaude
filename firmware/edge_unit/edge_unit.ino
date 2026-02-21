/**
 * edge_unit.ino  –  Phase 2 Step1: E220基本通信
 *
 * 実装内容:
 *   - E220ドライバ（モード切替・設定読み取り）
 *   - AUX割り込みによるコマンド受信基盤
 *   - 起動時 AUX=HIGH 待ち（E220初期化完了確認後に割り込み有効化）
 *   - K(Ping) / V(Version) / H(HW情報) / E(Error) 応答
 *
 * 未実装（後続Step）:
 *   - P/C センサー応答（Step2）
 *   - デュアルコア センサーサンプリング（Step3）
 *   - 設定UI・E220設定変更（Step4）
 *   - ローカルテストモード（Step5）
 *
 * 通信フォーマット（ペイロードレベル）:
 *   GW→エッジ: [CMD]['\r']['\n']
 *   エッジ→GW: [MY_ADDR_H][MY_ADDR_L][CMD][data...]['\r']['\n']
 *   ※ E220固定アドレスモードのため、UARTへの実書き込みは
 *     [GW_ADDR_H=0x00][GW_ADDR_L=0x00][CH] + ペイロード
 */

// ===== Pin Assignment (CLAUDE.md 準拠) =====
static const uint8_t PIN_LORA_M0   =  2;
static const uint8_t PIN_LORA_M1   =  3;
static const uint8_t PIN_LORA_TX   =  4;  // Serial2 TX (UART1)
static const uint8_t PIN_LORA_RX   =  5;  // Serial2 RX (UART1)
static const uint8_t PIN_LORA_AUX  =  6;
static const uint8_t PIN_BTN_SEL   =  7;  // 灰ボタン: 選択
static const uint8_t PIN_BTN_OK    =  8;  // 赤ボタン: 決定
static const uint8_t PIN_BTN_BACK  =  9;  // 黒ボタン: 戻る
static const uint8_t PIN_DIP1      = 10;  // パトライト監視
static const uint8_t PIN_DIP2      = 11;  // 電流監視
static const uint8_t PIN_DIP3      = 12;  // 予備
static const uint8_t PIN_DIP4      = 13;  // 予備
static const uint8_t PIN_LED_D3    = 14;  // センサー異常
static const uint8_t PIN_LED_D4    = 15;  // モード表示
static const uint8_t PIN_LED_D2    = 22;  // LoRa通信状態
static const uint8_t PIN_LED_D1    = 28;  // ハートビート

// ===== Firmware Version =====
static const uint8_t FW_MAJOR = 1;
static const uint8_t FW_MINOR = 0;
static const uint8_t FW_PATCH = 0;

// ===== Error Codes =====
static const uint8_t ERR_SENSOR_FAIL  = 0x01;  // センサー異常
static const uint8_t ERR_UNKNOWN_CMD  = 0x02;  // 未対応コマンド
static const uint8_t ERR_TIMEOUT      = 0x03;  // 処理タイムアウト

// ===== LoRa通信タイムアウト =====
// GWが1分周期でポーリングするため、5分無通信でLoRa断線とみなす
static const uint32_t LORA_COMMS_TIMEOUT_MS = 5UL * 60UL * 1000UL;

// ===== E220設定（起動時にE2PROMから読み取り） =====
struct E220Config {
    uint8_t addH;     // ADDH（アドレス上位）
    uint8_t addL;     // ADDL（アドレス下位）
    uint8_t netId;    // NETID
    uint8_t reg0;     // REG0: bits[5:3]=UARTボーレート, bits[2:0]=無線速度
    uint8_t reg1;     // REG1: bits[5:4]=サブパケット, bit3=ノイズ, bits[1:0]=送信電力
    uint8_t channel;  // チャンネル（0x00〜0x50）
};

// デフォルト値（E220設定読み取り失敗時のフォールバック）
static E220Config g_e220 = {0x01, 0x01, 0x00, 0x00, 0x00, 18};

// ===== AUX割り込みフラグ =====
static volatile bool g_auxRise = false;

static void onAuxRise() {
    g_auxRise = true;
}

// ===== 状態変数 =====
static uint32_t g_lastHbMs   = 0;    // D1点滅タイミング管理
static bool     g_ledD1State = false;
static uint32_t g_lastRxMs   = 0;    // 最終LoRa受信時刻
static bool     g_loraOk     = false; // LoRa通信状態フラグ

// ===== E220 モード切替 =====
static void e220WaitAuxHigh() {
    while (digitalRead(PIN_LORA_AUX) == LOW) delay(1);
}

// 通常動作モード（M0=LOW, M1=LOW）
static void e220SetNormalMode() {
    digitalWrite(PIN_LORA_M0, LOW);
    digitalWrite(PIN_LORA_M1, LOW);
    delay(2);
    e220WaitAuxHigh();
}

// 設定モード（M0=HIGH, M1=HIGH）
static void e220SetConfigMode() {
    digitalWrite(PIN_LORA_M0, HIGH);
    digitalWrite(PIN_LORA_M1, HIGH);
    delay(2);
    e220WaitAuxHigh();
}

// ===== E220 設定読み取り =====
// コマンド: C1 00 06 → レジスタ0x00〜0x05を6バイト読み取る
// レスポンス: [C1][00][06][ADDH][ADDL][NETID][REG0][REG1][CHAN] (9バイト)
static bool e220ReadConfig(E220Config &cfg) {
    e220SetConfigMode();
    delay(20);

    // 受信バッファをフラッシュ
    while (Serial2.available()) Serial2.read();

    // 読み取りコマンド送信
    uint8_t cmd[3] = {0xC1, 0x00, 0x06};
    Serial2.write(cmd, sizeof(cmd));
    Serial2.flush();

    // レスポンス待ち（ヘッダ3バイト + データ6バイト = 9バイト）
    uint32_t t = millis();
    while (Serial2.available() < 9) {
        if (millis() - t > 500) {
            e220SetNormalMode();
            return false;
        }
        delay(1);
    }

    uint8_t buf[9];
    for (int i = 0; i < 9; i++) buf[i] = Serial2.read();

    // ヘッダ確認
    if (buf[0] != 0xC1) {
        e220SetNormalMode();
        return false;
    }

    cfg.addH    = buf[3];
    cfg.addL    = buf[4];
    cfg.netId   = buf[5];
    cfg.reg0    = buf[6];
    cfg.reg1    = buf[7];
    cfg.channel = buf[8];

    e220SetNormalMode();
    delay(20);
    return true;
}

// ===== GWへのレスポンス送信 =====
// E220固定アドレスモード: [GW_ADDR_H=0x00][GW_ADDR_L=0x00][CH] + ペイロード
static void sendToGW(const uint8_t *payload, uint8_t len) {
    uint8_t header[3] = {0x00, 0x00, g_e220.channel};
    Serial2.write(header, sizeof(header));
    Serial2.write(payload, len);
    Serial2.flush();
}

// 受信成功時の共通処理（D2点灯・タイムスタンプ更新）
static void onRxSuccess() {
    g_loraOk   = true;
    g_lastRxMs = millis();
    digitalWrite(PIN_LED_D2, HIGH);
}

// ===== コマンドハンドラ =====

// K: Ping応答（5バイト）
// [MY_ADDR_H][MY_ADDR_L]['K']['\r']['\n']
static void cmdPing() {
    uint8_t resp[] = {g_e220.addH, g_e220.addL, 'K', '\r', '\n'};
    sendToGW(resp, sizeof(resp));
    onRxSuccess();
    Serial.println("[K] Ping response sent");
}

// V: バージョン応答（8バイト）
// [MY_ADDR_H][MY_ADDR_L]['V'][MAJOR][MINOR][PATCH]['\r']['\n']
static void cmdVersion() {
    uint8_t resp[] = {
        g_e220.addH, g_e220.addL, 'V',
        FW_MAJOR, FW_MINOR, FW_PATCH,
        '\r', '\n'
    };
    sendToGW(resp, sizeof(resp));
    onRxSuccess();
    Serial.printf("[V] Version %d.%d.%d sent\n", FW_MAJOR, FW_MINOR, FW_PATCH);
}

// H: ハードウェア情報応答（11バイト）
// [MY_ADDR_H][MY_ADDR_L]['H'][DIP][E220_ADDR_H][E220_ADDR_L][E220_CH][AIR_RATE][TX_POWER]['\r']['\n']
// DIP: bit0=パトライト, bit1=電流, bit2=DIP3予備, bit3=DIP4予備（LOW=ON）
static void cmdHwInfo() {
    uint8_t dip = 0;
    if (digitalRead(PIN_DIP1) == LOW) dip |= 0x01;
    if (digitalRead(PIN_DIP2) == LOW) dip |= 0x02;
    if (digitalRead(PIN_DIP3) == LOW) dip |= 0x04;
    if (digitalRead(PIN_DIP4) == LOW) dip |= 0x08;

    uint8_t airRate = g_e220.reg0 & 0x07;  // REG0 bits[2:0]: 無線速度
    uint8_t txPower = g_e220.reg1 & 0x03;  // REG1 bits[1:0]: 送信電力

    uint8_t resp[] = {
        g_e220.addH, g_e220.addL, 'H',
        dip,
        g_e220.addH, g_e220.addL,
        g_e220.channel,
        airRate,
        txPower,
        '\r', '\n'
    };
    sendToGW(resp, sizeof(resp));
    onRxSuccess();
    Serial.printf("[H] HW info: DIP=0x%02X addr=0x%02X%02X ch=%d ar=%d pwr=%d\n",
        dip, g_e220.addH, g_e220.addL, g_e220.channel, airRate, txPower);
}

// E: エラー応答（6バイト）
// [MY_ADDR_H][MY_ADDR_L]['E'][ERROR_CODE]['\r']['\n']
static void cmdError(uint8_t code) {
    uint8_t resp[] = {g_e220.addH, g_e220.addL, 'E', code, '\r', '\n'};
    sendToGW(resp, sizeof(resp));
    Serial.printf("[E] Error 0x%02X sent\n", code);
}

// ===== コマンド受信・ディスパッチ =====
static void processCommand() {
    // CRLFまで読み取る（最大32バイト、最大100ms待ち）
    uint8_t  buf[32];
    int      len = 0;
    uint32_t t   = millis();

    while (len < (int)sizeof(buf)) {
        if (Serial2.available()) {
            uint8_t b = Serial2.read();
            buf[len++] = b;
            // CRLF検出でフレーム確定
            if (len >= 3 && buf[len - 2] == '\r' && buf[len - 1] == '\n') break;
        }
        if (millis() - t > 100) break;
    }

    if (len < 3) return;  // フレーム不完全

    uint8_t cmd = buf[0];
    Serial.printf("[RX] cmd='%c' (%d bytes)\n", (char)cmd, len);

    switch (cmd) {
        case 'K': cmdPing();               break;
        case 'V': cmdVersion();            break;
        case 'H': cmdHwInfo();             break;
        case 'P': cmdError(ERR_UNKNOWN_CMD); break;  // Step2で実装
        case 'C': cmdError(ERR_UNKNOWN_CMD); break;  // Step2で実装
        default:  cmdError(ERR_UNKNOWN_CMD); break;
    }
}

// ===== Setup =====
void setup() {
    Serial.begin(115200);
    // USB接続待ち（最大5秒。USB未接続のまま起動する場合はタイムアウト後に続行）
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 5000) delay(10);
    Serial.printf("=== edge_unit v%d.%d.%d booting ===\n", FW_MAJOR, FW_MINOR, FW_PATCH);

    // ---- LED 初期化（全消灯） ----
    pinMode(PIN_LED_D1, OUTPUT); digitalWrite(PIN_LED_D1, LOW);
    pinMode(PIN_LED_D2, OUTPUT); digitalWrite(PIN_LED_D2, LOW);
    pinMode(PIN_LED_D3, OUTPUT); digitalWrite(PIN_LED_D3, LOW);
    pinMode(PIN_LED_D4, OUTPUT); digitalWrite(PIN_LED_D4, LOW);

    // ---- ボタン・DIPスイッチ（プルアップ: LOW=ON） ----
    pinMode(PIN_BTN_SEL,  INPUT_PULLUP);
    pinMode(PIN_BTN_OK,   INPUT_PULLUP);
    pinMode(PIN_BTN_BACK, INPUT_PULLUP);
    pinMode(PIN_DIP1, INPUT_PULLUP);
    pinMode(PIN_DIP2, INPUT_PULLUP);
    pinMode(PIN_DIP3, INPUT_PULLUP);
    pinMode(PIN_DIP4, INPUT_PULLUP);

    // ---- E220 モード制御ピン ----
    pinMode(PIN_LORA_M0,  OUTPUT);
    pinMode(PIN_LORA_M1,  OUTPUT);
    pinMode(PIN_LORA_AUX, INPUT);

    // ---- Serial2 (E220 UART1: GPIO4/5) ----
    Serial2.setTX(PIN_LORA_TX);
    Serial2.setRX(PIN_LORA_RX);
    Serial2.begin(115200);

    // ---- E220初期化完了待ち（AUX=HIGH） ----
    // attachInterrupt() の前に必ずAUX=HIGHを確認すること（設計上の重要注意事項）
    Serial.println("Waiting for E220 init (AUX=HIGH)...");
    digitalWrite(PIN_LORA_M0, LOW);
    digitalWrite(PIN_LORA_M1, LOW);
    delay(100);
    e220WaitAuxHigh();
    Serial.println("E220 ready");

    // ---- E220 設定読み取り（E2PROMから） ----
    Serial.println("Reading E220 config from E2PROM...");
    if (e220ReadConfig(g_e220)) {
        Serial.printf("  ADDR=0x%02X%02X  CH=%d  REG0=0x%02X  REG1=0x%02X\n",
            g_e220.addH, g_e220.addL, g_e220.channel, g_e220.reg0, g_e220.reg1);
    } else {
        Serial.println("  WARNING: E220 config read failed. Using defaults.");
        Serial.printf("  Default: ADDR=0x%02X%02X  CH=%d\n",
            g_e220.addH, g_e220.addL, g_e220.channel);
    }

    // ---- DIPスイッチ状態表示 ----
    uint8_t dip = 0;
    if (digitalRead(PIN_DIP1) == LOW) dip |= 0x01;
    if (digitalRead(PIN_DIP2) == LOW) dip |= 0x02;
    Serial.printf("DIP: 0x%02X (%s%s)\n",
        dip,
        (dip & 0x01) ? "patlite " : "",
        (dip & 0x02) ? "current" : "");

    // ---- AUX割り込み有効化（AUX=HIGH確認後） ----
    attachInterrupt(digitalPinToInterrupt(PIN_LORA_AUX), onAuxRise, RISING);

    Serial.println("=== Ready. Waiting for GW commands. ===");
}

// ===== Main Loop (Core0) =====
void loop() {
    // --- AUX割り込み処理: コマンド受信 ---
    // AUX RISINGは受信時と送信完了時の両方で発生する。
    // Serial2.available() >= 3 で受信データあり（送信完了後はバッファ空）と区別。
    if (g_auxRise) {
        g_auxRise = false;
        if (Serial2.available() >= 3) {
            processCommand();
        }
    }

    // --- D1 ハートビートLED（500msトグル → 1秒周期点滅） ---
    // Step3でCore1生存確認を追加予定。現時点はCore0のみで点滅。
    uint32_t now = millis();
    if (now - g_lastHbMs >= 500) {
        g_lastHbMs   = now;
        g_ledD1State = !g_ledD1State;
        digitalWrite(PIN_LED_D1, g_ledD1State);
    }

    // --- D2 LoRa通信状態LED ---
    // 最終受信から LORA_COMMS_TIMEOUT_MS 以上経過したら消灯（LoRa断線とみなす）
    if (g_loraOk && (millis() - g_lastRxMs >= LORA_COMMS_TIMEOUT_MS)) {
        g_loraOk = false;
        digitalWrite(PIN_LED_D2, LOW);
        Serial.println("[D2] LoRa comms timeout -> D2 OFF");
    }
}

// ===== Core1 (Step3で実装予定) =====
// void setup1() { ... }
// void loop1()  { ... }
