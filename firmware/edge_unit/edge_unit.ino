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
 * 通常動作モードでは DESIRED_REG0 に設定したボーレート（9600bps）を使用する。
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
// E220-900T22S レジスタマップ（NETIDなし）:
//   0x00:ADDH  0x01:ADDL  0x02:REG0(SPED)  0x03:REG1  0x04:CHAN  0x05:REG3(OPTION)
static const uint8_t DESIRED_ADDH    = 0x01;   // machine=1
static const uint8_t DESIRED_ADDL    = 0x01;   // type=patlite
static const uint8_t DESIRED_REG0    = 0x64;   // 9600bps UART / 9375bps air (SF6/BW125kHz)
                                                //  bits[7:5]=011(9600bps) bits[4:0]=00100(SF6/BW125k)
static const uint8_t DESIRED_REG1    = 0x01;   // 200バイト / RSSI無効 / 13dBm(22S JPデフォルト)
                                                //  bits[7:6]=00(200B) bit5=0(RSSI無効) bits[3:0]=0001(13dBm)
static const uint8_t DESIRED_CHANNEL = 2;      // LoRaチャンネル (JP版: BW125kHz→921.0MHz, BW500kHz→921.2MHz, Zone A)
static const uint8_t DESIRED_REG3    = 0x40;   // 固定アドレスモード（bit6=1）

// ===== E220設定構造体 =====
// NETIDフィールドは存在しない（E220-900T22Sの実レジスタマップに準拠）
struct E220Config {
    uint8_t addH;     // ADDH (0x00)
    uint8_t addL;     // ADDL (0x01)
    uint8_t reg0;     // REG0: UART baud / air rate (0x02)  ※JP版: パリティフィールドなし
    uint8_t reg1;     // REG1: packet size / noise / TX power (0x03)
    uint8_t channel;  // CHAN: LoRaチャンネル (0x04)
    uint8_t reg3;     // REG3: 固定アドレス / RSSI / TX電力テーブル(22Lのみ) / WOR (0x05)
};

static E220Config g_e220 = {
    DESIRED_ADDH, DESIRED_ADDL,
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
// レジスタ 0x00〜0x05 の 6バイトを読み取る。
// レスポンス: [C1][00][06][ADDH][ADDL][REG0][REG1][CHAN][REG3] (9バイト)  ※NETIDなし
static bool e220ReadConfig(E220Config &cfg) {
    e220SetConfigMode();
    delay(100);

    serial2Begin(9600);
    while (Serial2.available()) Serial2.read();

    // C1 00 06: レジスタ0x00〜0x05を6バイト読み取り
    // レスポンス: [C1][00][06][ADDH][ADDL][REG0][REG1][CHAN][REG3] (9バイト)
    uint8_t cmd[3] = {0xC1, 0x00, 0x06};
    Serial2.write(cmd, sizeof(cmd));
    Serial2.flush();

    uint32_t t = millis();
    while (Serial2.available() < 9) {
        if (millis() - t > 500) {
            serial2Begin(9600);
            e220SetNormalMode();
            return false;
        }
        delay(1);
    }

    uint8_t buf[9];
    for (int i = 0; i < 9; i++) buf[i] = Serial2.read();

    serial2Begin(9600);
    e220SetNormalMode();
    delay(20);

    if (buf[0] != 0xC1) return false;

    cfg.addH    = buf[3];
    cfg.addL    = buf[4];
    cfg.reg0    = buf[5];
    cfg.reg1    = buf[6];
    cfg.channel = buf[7];
    cfg.reg3    = buf[8];
    return true;
}

// ===== E220 設定書き込み =====
// C0コマンド: E2PROMに永続書き込み。
// レスポンス: [C1][00][06][書き込んだデータ] (9バイト)  ※NETIDなし
static bool e220WriteConfig(const E220Config &cfg) {
    e220SetConfigMode();
    delay(100);

    serial2Begin(9600);
    while (Serial2.available()) Serial2.read();

    // C0 00 06: レジスタ0x00〜0x05に6バイト書き込み（E2PROM永続）
    // レスポンス: [C1][00][06][書き込んだデータ] (9バイト)
    uint8_t cmd[9] = {
        0xC0, 0x00, 0x06,
        cfg.addH, cfg.addL,
        cfg.reg0, cfg.reg1, cfg.channel, cfg.reg3
    };
    Serial2.write(cmd, sizeof(cmd));
    Serial2.flush();

    uint32_t t = millis();
    while (Serial2.available() < 9) {
        if (millis() - t > 500) {
            serial2Begin(9600);
            e220SetNormalMode();
            return false;
        }
        delay(1);
    }

    uint8_t buf[9];
    for (int i = 0; i < 9; i++) buf[i] = Serial2.read();

    serial2Begin(9600);
    e220SetNormalMode();
    delay(100);  // E2PROM書き込み完了待ち

    return (buf[0] == 0xC1);
}

// ===== 生データ HEX ダンプ =====
static void hexDump(const char *label, const uint8_t *data, int len) {
    Serial.printf("%s (%d bytes):", label, len);
    for (int i = 0; i < len; i++) Serial.printf(" %02X", data[i]);
    Serial.println();
}

// ===== GWへのレスポンス送信 =====
static void sendToGW(const uint8_t *payload, uint8_t len) {
    uint8_t header[3] = {0x00, 0x00, g_e220.channel};
    uint8_t full[35];
    memcpy(full, header, 3);
    memcpy(full + 3, payload, len);
    hexDump("[TX]", full, 3 + len);
    delay(100);  // GW E220のRX切替完了を待つ（即時応答問題対策）
    Serial2.write(header, sizeof(header));
    Serial2.write(payload, len);
    Serial2.flush();

    // AUX監視: E220がTXを受け付けるとAUXがLOWになるはず
    uint32_t t = millis();
    bool auxWentLow = false;
    while (millis() - t < 200) {
        if (digitalRead(PIN_LORA_AUX) == LOW) { auxWentLow = true; break; }
    }
    if (auxWentLow) {
        uint32_t lowDur = millis();
        while (digitalRead(PIN_LORA_AUX) == LOW && millis() - lowDur < 2000);
        Serial.printf("[TX] AUX LOW->HIGH (TX done, %lu ms)\n", millis() - lowDur);
    } else {
        Serial.println("[TX] AUX stayed HIGH (E220 did NOT accept TX data!)");
    }
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

    uint8_t airRate = g_e220.reg0 & 0x1F;  // JP版: bits[4:0] (SF/BW複合5ビット)
    uint8_t txPower = g_e220.reg1 & 0x0F;  // JP版 22S: bits[3:0]

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

    hexDump("[RX]", buf, len);
    uint8_t cmd = buf[0];
    Serial.printf("[RX] cmd='%c'\n", (char)cmd);

    switch (cmd) {
        case 'K': cmdPing();               break;
        case 'V': cmdVersion();            break;
        case 'H': cmdHwInfo();             break;
        case 'P': cmdError(ERR_UNKNOWN_CMD); break;  // Step2で実装
        case 'C': cmdError(ERR_UNKNOWN_CMD); break;  // Step2で実装
        default:  cmdError(ERR_UNKNOWN_CMD); break;
    }
}

// ===== E220 設定詳細ダンプ (JP版: E220-900T22S/22L(JP) firmware v2.0準拠) =====
// REG0: bits[7:5]=UARTボーレート, bits[4:0]=エアレート(SF/BW複合)  ※パリティフィールドなし
// REG1: bits[7:6]=パケットサイズ, bit5=RSSIノイズ, bit4=Reserved, bits[3:0]=TX電力(22S: 最大13dBm)
// REG3: bit7=RSSI付与, bit6=送信モード(1=通常/固定アドレス), bit5=TX電力テーブル(22Lのみ), bits[2:0]=WOR
static void e220PrintConfig(const E220Config &cfg, const char *label) {
    static const char *BAUD_STR[] = {"1200","2400","4800","9600","19200","38400","57600","115200"};
    static const char *PKT_STR[]  = {"200","128","64","32"};
    static const char *BW_STR[]   = {"BW125k","BW250k","BW500k","??"};
    static const char *WOR_STR[]  = {"500","1000","1500","2000","2500","3000","??","??"};
    // JP版 22S TX電力: bits[3:0]=0→未定義, 1→13dBm(default), 2→7dBm, 3→0dBm, 4-15→1-12dBm
    static const int8_t PWR_DBM[] = {-99,13,7,0,1,2,3,4,5,6,7,8,9,10,11,12};
    // JP版 BW開始周波数
    static const float  BW_BASE[] = {920.6f, 920.7f, 920.8f};

    uint8_t raw[6] = {cfg.addH, cfg.addL, cfg.reg0, cfg.reg1, cfg.channel, cfg.reg3};
    hexDump(label, raw, 6);

    uint8_t baudBits  = (cfg.reg0 >> 5) & 0x07;
    uint8_t airBits   =  cfg.reg0 & 0x1F;            // JP版: bits[4:0] = SF/BW複合
    uint8_t bwIdx     =  airBits & 0x03;              // bits[1:0]: 0=BW125k, 1=BW250k, 2=BW500k
    uint8_t sf        = ((airBits >> 2) & 0x07) + 5; // bits[4:2]+5 = SF5〜SF11
    uint8_t pktBits   = (cfg.reg1 >> 6) & 0x03;
    bool    rssiNoise = (cfg.reg1 >> 5) & 0x01;
    uint8_t pwrBits   =  cfg.reg1 & 0x0F;            // JP版 22S: bits[3:0]
    bool    rssiApp   = (cfg.reg3 >> 7) & 0x01;
    bool    fixedMode = (cfg.reg3 >> 6) & 0x01;
    bool    txTblB    = (cfg.reg3 >> 5) & 0x01;      // JP版 22L送信出力テーブル選択
    uint8_t worBits   =  cfg.reg3 & 0x07;            // JP版: bits[2:0]

    // エアレート計算: Rb = SF * (BW / 2^SF) * 4/5
    static const uint32_t BW_HZ[] = {125000, 250000, 500000, 0};
    float rb = (bwIdx < 3 && sf >= 5 && sf <= 12)
               ? (float)sf * BW_HZ[bwIdx] / (1UL << sf) * 0.8f : 0;
    float freq = (bwIdx < 3) ? BW_BASE[bwIdx] + cfg.channel * 0.2f : 0;

    Serial.printf("  Addr        : 0x%02X%02X\n", cfg.addH, cfg.addL);
    Serial.printf("  Channel     : %d (%.1f MHz, %s)\n",
                  cfg.channel, freq, BW_STR[bwIdx < 3 ? bwIdx : 3]);
    Serial.printf("  UART baud   : %s bps  [REG0 b7:5=%d]\n", BAUD_STR[baudBits], baudBits);
    Serial.printf("  Air rate    : %.0f bps  SF%d/%s [REG0 b4:0=0x%02X]\n",
                  rb, sf, BW_STR[bwIdx < 3 ? bwIdx : 3], airBits);
    Serial.printf("  Pkt size    : %s B    [REG1 b7:6=%d]\n", PKT_STR[pktBits], pktBits);
    Serial.printf("  RSSI noise  : %s      [REG1 b5]\n", rssiNoise ? "ENABLED" : "disabled");
    if (pwrBits == 0)
        Serial.printf("  TX power    : N/A(0=未定義) [REG1 b3:0=0x0]\n");
    else
        Serial.printf("  TX power    : %ddBm        [REG1 b3:0=0x%X]\n", PWR_DBM[pwrBits], pwrBits);
    Serial.printf("  RSSI append : %s      [REG3 b7]\n", rssiApp ? "ENABLED" : "disabled");
    Serial.printf("  TX mode     : %s  [REG3 b6]\n", fixedMode ? "通常(固定アドレス)" : "透過");
    Serial.printf("  TX tbl(22L) : %s     [REG3 b5]\n", txTblB ? "Table B" : "Table A");
    Serial.printf("  WOR period  : %s ms   [REG3 b2:0=%d]\n", WOR_STR[worBits < 8 ? worBits : 6], worBits);
}

// ===== E220 設定チェック・自動書き込み =====
static void e220ConfigureIfNeeded() {
    Serial.println("Reading E220 config (9600bps config mode)...");
    E220Config current = {};
    bool readOk = e220ReadConfig(current);

    if (readOk) {
        e220PrintConfig(current, "[E220 current]");

        bool match = (current.addH    == DESIRED_ADDH    &&
                      current.addL    == DESIRED_ADDL    &&
                      current.channel == DESIRED_CHANNEL &&
                      current.reg0    == DESIRED_REG0    &&
                      current.reg1    == DESIRED_REG1    &&
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
        DESIRED_ADDH, DESIRED_ADDL,
        DESIRED_REG0, DESIRED_REG1, DESIRED_CHANNEL, DESIRED_REG3
    };
    if (e220WriteConfig(desired)) {
        g_e220 = desired;
        e220PrintConfig(g_e220, "[E220 after write]");
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

    // ---- Serial2 起動（通常動作モード: 9600bps） ----
    serial2Begin(9600);

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
