/**
 * edge_unit.ino  –  Phase 2 Step1: E220基本通信
 *
 * 実装内容:
 *   - E220ドライバ（モード切替・設定読み取り・設定書き込み）
 *   - AUX割り込みによるコマンド受信基盤
 *   - 起動時 AUX=HIGH 待ち（E220初期化完了確認後に割り込み有効化）
 *   - K(Ping) / V(Version) / H(HW情報) / E(Error) 応答
 *   - 初回起動時 E220 自動設定（DESIRED_* 定数に基づく）
 *
 * 未実装（後続Step）:
 *   - P/C センサー応答（Step2）
 *   - デュアルコア センサーサンプリング（Step3）
 *   - 設定UI・E220設定変更（Step4）
 *   - ローカルテストモード（Step5）
 *
 * E220 コンフィグモード(M0=HIGH,M1=HIGH)では UART は常に 9600bps 固定。
 * 通常動作モードでは DESIRED_REG0 に設定したボーレート（115200bps）を使用する。
 */

// ===== Pin Assignment (CLAUDE.md 準拠) =====
static const uint8_t PIN_LORA_M0   =  2;
static const uint8_t PIN_LORA_M1   =  3;
static const uint8_t PIN_LORA_TX   =  4;  // Serial2 TX (UART1)
static const uint8_t PIN_LORA_RX   =  5;  // Serial2 RX (UART1)
static const uint8_t PIN_LORA_AUX  =  6;
static const uint8_t PIN_BTN_SEL   =  7;
static const uint8_t PIN_BTN_OK    =  8;
static const uint8_t PIN_BTN_BACK  =  9;
static const uint8_t PIN_DIP1      = 10;
static const uint8_t PIN_DIP2      = 11;
static const uint8_t PIN_DIP3      = 12;
static const uint8_t PIN_DIP4      = 13;
static const uint8_t PIN_LED_D3    = 14;
static const uint8_t PIN_LED_D4    = 15;
static const uint8_t PIN_LED_D2    = 22;
static const uint8_t PIN_LED_D1    = 28;

// ===== Firmware Version =====
static const uint8_t FW_MAJOR = 1;
static const uint8_t FW_MINOR = 0;
static const uint8_t FW_PATCH = 0;

// ===== Error Codes =====
static const uint8_t ERR_SENSOR_FAIL = 0x01;
static const uint8_t ERR_UNKNOWN_CMD = 0x02;
static const uint8_t ERR_TIMEOUT     = 0x03;

// ===== LoRa通信タイムアウト =====
static const uint32_t LORA_COMMS_TIMEOUT_MS = 5UL * 60UL * 1000UL;

// ===== このユニットに書き込みたい E220 設定値 =====
// ユニットごとに DESIRED_ADDH/ADDL を変更すること（0xMMTT形式）
static const uint8_t DESIRED_ADDH    = 0x01;   // machine=1
static const uint8_t DESIRED_ADDL    = 0x01;   // type=patlite
static const uint8_t DESIRED_NETID   = 0x00;
static const uint8_t DESIRED_REG0    = 0xE4;   // 115200bps / 8N1 / 9.6kbps air
                                                //  bits[7:5]=111(115200) [4:3]=00(8N1) [2:0]=100(9.6k)
static const uint8_t DESIRED_REG1    = 0x00;   // 200バイト / RSSI無効 / 22dBm
static const uint8_t DESIRED_CHANNEL = 18;     // LoRaチャンネル
static const uint8_t DESIRED_REG3    = 0x40;   // 固定アドレスモード（bit6=1）

// ===== E220設定構造体 =====
struct E220Config {
    uint8_t addH;     // ADDH
    uint8_t addL;     // ADDL
    uint8_t netId;    // NETID
    uint8_t reg0;     // UART baud / parity / air rate
    uint8_t reg1;     // packet size / noise / TX power
    uint8_t channel;  // LoRaチャンネル
    uint8_t reg3;     // 固定アドレス / RSSI / LBT / WOR
};

static E220Config g_e220 = {
    DESIRED_ADDH, DESIRED_ADDL, DESIRED_NETID,
    DESIRED_REG0, DESIRED_REG1, DESIRED_CHANNEL, DESIRED_REG3
};

// ===== AUX割り込みフラグ =====
static volatile bool g_auxRise = false;
static void onAuxRise() { g_auxRise = true; }

// ===== 状態変数 =====
static uint32_t g_lastHbMs   = 0;
static bool     g_ledD1State = false;
static uint32_t g_lastRxMs   = 0;
static bool     g_loraOk     = false;

// ===== Serial2 初期化ヘルパー =====
static void serial2Begin(uint32_t baud) {
    Serial2.end();
    Serial2.setTX(PIN_LORA_TX);
    Serial2.setRX(PIN_LORA_RX);
    Serial2.begin(baud);
    delay(50);
}

// ===== E220 モード切替 =====
static void e220WaitAuxHigh() {
    while (digitalRead(PIN_LORA_AUX) == LOW) delay(1);
}

static void e220SetNormalMode() {
    digitalWrite(PIN_LORA_M0, LOW);
    digitalWrite(PIN_LORA_M1, LOW);
    delay(2);
    e220WaitAuxHigh();
}

static void e220SetConfigMode() {
    digitalWrite(PIN_LORA_M0, HIGH);
    digitalWrite(PIN_LORA_M1, HIGH);
    delay(2);
    e220WaitAuxHigh();
}

// ===== E220 設定読み取り =====
// E220コンフィグモードのUARTは 9600bps 固定。
// レジスタ 0x00〜0x06 の 7バイトを読み取る。
// レスポンス: [C1][00][07][ADDH][ADDL][NETID][REG0][REG1][CHAN][REG3] (10バイト)
static bool e220ReadConfig(E220Config &cfg) {
    e220SetConfigMode();
    delay(100);

    serial2Begin(9600);
    while (Serial2.available()) Serial2.read();

    uint8_t cmd[3] = {0xC1, 0x00, 0x07};
    Serial2.write(cmd, sizeof(cmd));
    Serial2.flush();

    uint32_t t = millis();
    while (Serial2.available() < 10) {
        if (millis() - t > 500) {
            serial2Begin(115200);
            e220SetNormalMode();
            return false;
        }
        delay(1);
    }

    uint8_t buf[10];
    for (int i = 0; i < 10; i++) buf[i] = Serial2.read();

    serial2Begin(115200);
    e220SetNormalMode();
    delay(20);

    if (buf[0] != 0xC1) return false;

    cfg.addH    = buf[3];
    cfg.addL    = buf[4];
    cfg.netId   = buf[5];
    cfg.reg0    = buf[6];
    cfg.reg1    = buf[7];
    cfg.channel = buf[8];
    cfg.reg3    = buf[9];
    return true;
}

// ===== E220 設定書き込み =====
// C0コマンド: E2PROMに永続書き込み。
// レスポンス: [C1][00][07][書き込んだデータ] (10バイト)
static bool e220WriteConfig(const E220Config &cfg) {
    e220SetConfigMode();
    delay(100);

    serial2Begin(9600);
    while (Serial2.available()) Serial2.read();

    uint8_t cmd[10] = {
        0xC0, 0x00, 0x07,
        cfg.addH, cfg.addL, cfg.netId,
        cfg.reg0, cfg.reg1, cfg.channel,
        cfg.reg3
    };
    Serial2.write(cmd, sizeof(cmd));
    Serial2.flush();

    uint32_t t = millis();
    while (Serial2.available() < 10) {
        if (millis() - t > 500) {
            serial2Begin(115200);
            e220SetNormalMode();
            return false;
        }
        delay(1);
    }

    uint8_t buf[10];
    for (int i = 0; i < 10; i++) buf[i] = Serial2.read();

    serial2Begin(115200);
    e220SetNormalMode();
    delay(100);  // E2PROM書き込み完了待ち

    return (buf[0] == 0xC1);
}

// ===== GWへのレスポンス送信 =====
static void sendToGW(const uint8_t *payload, uint8_t len) {
    uint8_t header[3] = {0x00, 0x00, g_e220.channel};
    Serial2.write(header, sizeof(header));
    Serial2.write(payload, len);
    Serial2.flush();
}

static void onRxSuccess() {
    g_loraOk   = true;
    g_lastRxMs = millis();
    digitalWrite(PIN_LED_D2, HIGH);
}

// ===== コマンドハンドラ =====

static void cmdPing() {
    uint8_t resp[] = {g_e220.addH, g_e220.addL, 'K', '\r', '\n'};
    sendToGW(resp, sizeof(resp));
    onRxSuccess();
    Serial.println("[K] Ping response sent");
}

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

static void cmdHwInfo() {
    uint8_t dip = 0;
    if (digitalRead(PIN_DIP1) == LOW) dip |= 0x01;
    if (digitalRead(PIN_DIP2) == LOW) dip |= 0x02;
    if (digitalRead(PIN_DIP3) == LOW) dip |= 0x04;
    if (digitalRead(PIN_DIP4) == LOW) dip |= 0x08;

    uint8_t airRate = g_e220.reg0 & 0x07;
    uint8_t txPower = g_e220.reg1 & 0x03;

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

static void cmdError(uint8_t code) {
    uint8_t resp[] = {g_e220.addH, g_e220.addL, 'E', code, '\r', '\n'};
    sendToGW(resp, sizeof(resp));
    Serial.printf("[E] Error 0x%02X sent\n", code);
}

// ===== コマンド受信・ディスパッチ =====
static void processCommand() {
    uint8_t  buf[32];
    int      len = 0;
    uint32_t t   = millis();

    while (len < (int)sizeof(buf)) {
        if (Serial2.available()) {
            uint8_t b = Serial2.read();
            buf[len++] = b;
            if (len >= 3 && buf[len - 2] == '\r' && buf[len - 1] == '\n') break;
        }
        if (millis() - t > 100) break;
    }

    if (len < 3) return;

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

// ===== E220 設定チェック・自動書き込み =====
static void e220ConfigureIfNeeded() {
    Serial.println("Reading E220 config (9600bps config mode)...");
    E220Config current = {};
    bool readOk = e220ReadConfig(current);

    if (readOk) {
        Serial.printf("  Read OK: ADDR=0x%02X%02X  CH=%d  REG0=0x%02X  REG3=0x%02X\n",
            current.addH, current.addL, current.channel, current.reg0, current.reg3);

        bool match = (current.addH    == DESIRED_ADDH    &&
                      current.addL    == DESIRED_ADDL    &&
                      current.channel == DESIRED_CHANNEL &&
                      current.reg0    == DESIRED_REG0    &&
                      current.reg3    == DESIRED_REG3);

        if (match) {
            Serial.println("  Settings OK. No write needed.");
            g_e220 = current;
            return;
        }
        Serial.println("  Settings differ from desired. Writing...");
    } else {
        Serial.println("  Read FAILED. Attempting to write defaults...");
    }

    // 設定書き込み
    E220Config desired = {
        DESIRED_ADDH, DESIRED_ADDL, DESIRED_NETID,
        DESIRED_REG0, DESIRED_REG1, DESIRED_CHANNEL, DESIRED_REG3
    };
    if (e220WriteConfig(desired)) {
        g_e220 = desired;
        Serial.printf("  Write OK: ADDR=0x%02X%02X  CH=%d\n",
            g_e220.addH, g_e220.addL, g_e220.channel);
    } else {
        Serial.println("  Write FAILED. Using desired values as-is.");
        g_e220 = desired;
    }
}

// ===== Setup =====
void setup() {
    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 5000) delay(10);
    Serial.printf("=== edge_unit v%d.%d.%d booting ===\n", FW_MAJOR, FW_MINOR, FW_PATCH);

    // ---- LED 初期化 ----
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

    // ---- Serial2 起動（通常動作モード: 115200bps） ----
    serial2Begin(115200);

    // ---- E220初期化完了待ち（AUX=HIGH確認） ----
    Serial.println("Waiting for E220 init (AUX=HIGH)...");
    digitalWrite(PIN_LORA_M0, LOW);
    digitalWrite(PIN_LORA_M1, LOW);
    delay(100);
    e220WaitAuxHigh();
    Serial.println("E220 hardware ready");

    // ---- E220 設定確認・必要なら書き込み ----
    e220ConfigureIfNeeded();

    // ---- DIPスイッチ状態表示 ----
    uint8_t dip = 0;
    if (digitalRead(PIN_DIP1) == LOW) dip |= 0x01;
    if (digitalRead(PIN_DIP2) == LOW) dip |= 0x02;
    Serial.printf("DIP: 0x%02X (%s%s)\n",
        dip,
        (dip & 0x01) ? "patlite " : "",
        (dip & 0x02) ? "current" : "");

    // ---- AUX割り込み有効化 ----
    attachInterrupt(digitalPinToInterrupt(PIN_LORA_AUX), onAuxRise, RISING);

    Serial.println("=== Ready. Waiting for GW commands. ===");
}

// ===== Main Loop (Core0) =====
void loop() {
    if (g_auxRise) {
        g_auxRise = false;
        if (Serial2.available() >= 3) {
            processCommand();
        }
    }

    // D1 ハートビート（Step3でCore1生存確認を追加予定）
    uint32_t now = millis();
    if (now - g_lastHbMs >= 500) {
        g_lastHbMs   = now;
        g_ledD1State = !g_ledD1State;
        digitalWrite(PIN_LED_D1, g_ledD1State);
    }

    // D2 LoRa通信状態
    if (g_loraOk && (millis() - g_lastRxMs >= LORA_COMMS_TIMEOUT_MS)) {
        g_loraOk = false;
        digitalWrite(PIN_LED_D2, LOW);
        Serial.println("[D2] LoRa comms timeout -> D2 OFF");
    }
}

// ===== Core1 (Step3で実装予定) =====
// void setup1() { ... }
// void loop1()  { ... }
