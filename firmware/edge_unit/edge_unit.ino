/**
 * edge_unit.ino  –  Phase 2 Step3: センサー統合（デュアルコア）
 *
 * 実装内容:
 *   - E220ドライバ（モード切替・設定読み取り・設定書き込み）
 *   - AUX割り込みによるコマンド受信基盤
 *   - 起動時 AUX=HIGH 待ち（E220初期化完了確認後に割り込み有効化）
 *   - K(Ping) / V(Version) / H(HW情報) / E(Error) 応答
 *   - P(Patlite): Core1がTSL2561を常時サンプリング（13ms/GAIN_1X + 1.5秒maxウィンドウ）
 *   - C(Current): Core1がMCP3208を~1kHzサンプリング → RMS計算 → 実電流値で応答
 *   - 初回起動時 E220 自動設定（DESIRED_ADDH 定数 + DIPから自動設定される ADDL に基づく）
 *   - DIPスイッチによるユニット種別判定
 *   - Core1/Core0 mutex_t コア間共有データ保護
 *   - D1ハートビート: Core0+Core1の両方が生きているときのみ点滅
 *
 * 未実装（後続Step）:
 *   - 設定UI・E220設定変更（Step4）
 *   - ローカルテストモード（Step5）
 *
 * E220 コンフィグモード(M0=HIGH,M1=HIGH)では UART は常に 9600bps 固定。
 * 通常動作モードでは DESIRED_REG0 に設定したボーレート（9600bps）を使用する。
 */

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <pico/mutex.h>

// ===== Unit Type =====
static const uint8_t UNIT_PATLITE = 0x01;
static const uint8_t UNIT_CURRENT = 0x02;

static uint8_t g_unit_type = 0;

// ===== I2C / TCA9548A =====
static const uint8_t PIN_I2C0_SDA = 20;
static const uint8_t PIN_I2C0_SCL = 21;
static const uint8_t TCA_ADDR     = 0x70;

// ===== コア間共有データ =====
struct SharedData {
    float    patlite_max[3];   // TCA CH0/1/2 の直近1.5秒max値 [lux]
    float    current_rms;      // RMS電流値 [A]
    bool     sensor_error;     // センサー異常フラグ
    uint32_t core1_heartbeat;  // Core1生存確認カウンタ
};
static SharedData g_shared = {};
static mutex_t    g_mutex;

// ===== TSL2561センサー =====
static Adafruit_TSL2561_Unified g_tsl0(TSL2561_ADDR_FLOAT, 1000);
static Adafruit_TSL2561_Unified g_tsl1(TSL2561_ADDR_FLOAT, 1001);
static Adafruit_TSL2561_Unified g_tsl2(TSL2561_ADDR_FLOAT, 1002);
static Adafruit_TSL2561_Unified *g_tsl[3] = {&g_tsl0, &g_tsl1, &g_tsl2};
static bool g_tsl_ok[3] = {};

// ===== 1.5秒maxウィンドウ（Core1ローカル） =====
static const uint32_t PATLITE_WINDOW_MS = 1500;
static float    g_patlite_local_max[3]  = {};
static uint32_t g_patlite_expire_ms[3]  = {};

// ===== MCP3208 電流計測 =====
// 回路: CTL-24-CLS → R1(100Ω) → 計測点 (DCバイアス1.65V) → MCP3208 CH0
// 変換: Io(Arms) = Vac_rms(V) × 20   ※ n/RL = 2000/100 = 20, K≈1 (Io≥10A)
// クリップ: ~23A でADC飽和（22A超の検知用途には問題なし）
static const uint8_t  PIN_SPI_MISO      = 16;
static const uint8_t  PIN_SPI_CS        = 17;
static const uint8_t  PIN_SPI_SCK       = 18;
static const uint8_t  PIN_SPI_MOSI      = 19;
static const uint8_t  ADC_CURRENT_CH    = 0;      // MCP3208 CH0
static const int32_t  ADC_DC_OFFSET     = 2048;   // 1.65V / 3.3V × 4096
static const uint32_t CURRENT_WINDOW_MS = 100;    // RMSウィンドウ（50Hzで5周期）

// 電流計測（Core1ローカル）
static int64_t  g_current_sum_sq       = 0;
static uint32_t g_current_count        = 0;
static uint32_t g_current_window_start = 0;
static int64_t  g_current_sum          = 0;      // mean計算用
static int32_t  g_current_raw_min      = 32767;  // 診断用
static int32_t  g_current_raw_max      = 0;      // 診断用

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
static const uint8_t PIN_LED_D3    = 14;  // 黄: モード表示（設定/テストモード中点灯）
static const uint8_t PIN_LED_D4    = 15;  // 赤: センサー異常（エラー時点灯）
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

// ===== LEDフラッシュ時間 =====
static const uint32_t D1_FLASH_MS   = 100;    // ハートビートパルス幅 [ms]
static const uint32_t D1_PERIOD_MS  = 5000;   // ハートビート周期 [ms]
static const uint32_t D2_FLASH_MS   = 100;    // LoRa受信時のピカッ点灯時間 [ms]

// ===== このユニットに書き込みたい E220 設定値 =====
// ユニットごとに DESIRED_ADDH を変更すること（機械番号MM、0xMMTT形式）
// DESIRED_ADDL はDIPスイッチから自動設定（setup()内で g_unit_type を代入）
// E220-900T22S レジスタマップ（NETIDなし）:
//   0x00:ADDH  0x01:ADDL  0x02:REG0(SPED)  0x03:REG1  0x04:CHAN  0x05:REG3(OPTION)
static const uint8_t DESIRED_ADDH    = 0x01;   // machine=1（ユニットごとに変更すること）
static uint8_t       DESIRED_ADDL    = 0x00;   // DIPスイッチから自動設定（setup()で g_unit_type を代入）
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
static uint32_t g_lastHbMs    = 0;
static uint32_t g_d1FlashMs   = 0;  // D1点灯開始時刻（0=消灯中）
static uint32_t g_d2FlashMs   = 0;  // D2点灯開始時刻（0=消灯中）
static uint32_t g_lastCore1Hb = 0;

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
    g_d2FlashMs = millis();
    digitalWrite(PIN_LED_D2, HIGH);
}

// ===== TCA9548A チャンネル選択 =====
static void tcaSelect(uint8_t ch) {
    Wire.beginTransmission(TCA_ADDR);
    Wire.write(1 << ch);
    Wire.endTransmission();
}

// ===== lux値をuint16_tに変換（クランプ） =====
static uint16_t luxToU16(float f) {
    if (f < 0.0f)     return 0;
    if (f > 65535.0f) return 65535;
    return (uint16_t)f;
}

// ===== 1.5秒maxウィンドウ更新（Core1から呼ぶ） =====
static void updatePatliteMax(uint8_t ch, float lux) {
    uint32_t now = millis();
    if (lux > g_patlite_local_max[ch]) {
        g_patlite_local_max[ch] = lux;
        g_patlite_expire_ms[ch] = now + PATLITE_WINDOW_MS;
    } else if (now >= g_patlite_expire_ms[ch]) {
        g_patlite_local_max[ch] = lux;
        if (lux > 0.0f) g_patlite_expire_ms[ch] = now + PATLITE_WINDOW_MS;
    }
}

// ===== TSL2561 初期化（setup()内のCore0から呼ぶ） =====
static void initPatliteSensors() {
    bool any_error = false;
    for (uint8_t ch = 0; ch < 3; ch++) {
        tcaSelect(ch);
        delay(2);
        if (!g_tsl[ch]->begin()) {
            Serial.printf("  [ERROR] TSL2561 not found on TCA CH%d\n", ch);
            g_tsl_ok[ch] = false;
            any_error = true;
        } else {
            g_tsl[ch]->setGain(TSL2561_GAIN_1X);
            g_tsl[ch]->setIntegrationTime(TSL2561_INTEGRATIONTIME_13MS);
            g_tsl_ok[ch] = true;
            Serial.printf("  TSL2561 ready on TCA CH%d (13ms/1X)\n", ch);
        }
    }
    mutex_enter_blocking(&g_mutex);
    g_shared.sensor_error = any_error;
    mutex_exit(&g_mutex);
    if (any_error) digitalWrite(PIN_LED_D4, HIGH);
}

// ===== MCP3208 raw読み取り（single-ended） =====
static uint16_t readMCP3208_raw(uint8_t ch) {
    uint8_t tx1 = 0x06;                 // Start=1, SGL=1
    uint8_t tx2 = (ch & 0x07) << 6;    // D2 D1 D0 を上位3bitに
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(PIN_SPI_CS, LOW);
    SPI.transfer(tx1);
    uint16_t hi = SPI.transfer(tx2);
    uint16_t lo = SPI.transfer(0x00);
    digitalWrite(PIN_SPI_CS, HIGH);
    SPI.endTransaction();
    return (uint16_t)((hi & 0x0F) << 8) | lo;  // 12bit結果
}

// ===== コマンドハンドラ =====

static void cmdError(uint8_t code) {
    uint8_t resp[] = {g_e220.addH, g_e220.addL, 'E', code, '\r', '\n'};
    sendToGW(resp, sizeof(resp));
    Serial.printf("[E] Error 0x%02X sent\n", code);
}

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

static void cmdPatlite() {
    if (!(g_unit_type & UNIT_PATLITE)) {
        cmdError(ERR_SENSOR_FAIL);
        return;
    }
    float    local_max[3];
    bool     sensor_err;
    mutex_enter_blocking(&g_mutex);
    local_max[0] = g_shared.patlite_max[0];
    local_max[1] = g_shared.patlite_max[1];
    local_max[2] = g_shared.patlite_max[2];
    sensor_err   = g_shared.sensor_error;
    mutex_exit(&g_mutex);

    if (sensor_err) { cmdError(ERR_SENSOR_FAIL); return; }

    uint16_t red = luxToU16(local_max[0]);
    uint16_t yel = luxToU16(local_max[1]);
    uint16_t grn = luxToU16(local_max[2]);
    uint8_t resp[] = {
        g_e220.addH, g_e220.addL, 'P',
        (uint8_t)(red >> 8), (uint8_t)(red & 0xFF),
        (uint8_t)(yel >> 8), (uint8_t)(yel & 0xFF),
        (uint8_t)(grn >> 8), (uint8_t)(grn & 0xFF),
        '\r', '\n'
    };
    sendToGW(resp, sizeof(resp));
    onRxSuccess();
    Serial.printf("[P] Patlite: red=%.1f yel=%.1f grn=%.1f lux\n",
                  local_max[0], local_max[1], local_max[2]);
}

static void cmdCurrent() {
    if (!(g_unit_type & UNIT_CURRENT)) {
        cmdError(ERR_SENSOR_FAIL);
        return;
    }
    float rms;
    mutex_enter_blocking(&g_mutex);
    rms = g_shared.current_rms;
    mutex_exit(&g_mutex);

    uint16_t val = (uint16_t)(rms * 100.0f);  // 0.01A単位
    uint8_t resp[] = {
        g_e220.addH, g_e220.addL, 'C',
        (uint8_t)(val >> 8), (uint8_t)(val & 0xFF),
        '\r', '\n'
    };
    sendToGW(resp, sizeof(resp));
    onRxSuccess();
    Serial.printf("[C] Current: %.2f A\n", rms);
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
        case 'P': cmdPatlite(); break;
        case 'C': cmdCurrent(); break;
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
    // ---- mutex は最初に初期化（Core1起動前に必須） ----
    mutex_init(&g_mutex);

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

    // ---- ユニット種別判定（DIPスイッチ） ----
    if (digitalRead(PIN_DIP1) == LOW) g_unit_type |= UNIT_PATLITE;
    if (digitalRead(PIN_DIP2) == LOW) g_unit_type |= UNIT_CURRENT;
    DESIRED_ADDL = g_unit_type;  // DIP値をそのまま E220 ADDL(TT部分)に反映
    Serial.printf("Unit type: 0x%02X (%s%s) -> ADDL=0x%02X\n",
        g_unit_type,
        (g_unit_type & UNIT_PATLITE) ? "patlite " : "",
        (g_unit_type & UNIT_CURRENT) ? "current"  : "",
        DESIRED_ADDL);

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

    // ---- パトライトユニット: I2C + TSL2561 初期化 ----
    if (g_unit_type & UNIT_PATLITE) {
        Serial.println("Init I2C (Wire) for TCA9548A + TSL2561...");
        Wire.setSDA(PIN_I2C0_SDA);
        Wire.setSCL(PIN_I2C0_SCL);
        Wire.begin();
        Serial.println("Init TSL2561 sensors...");
        initPatliteSensors();
    }

    // ---- 電流ユニット: SPI + MCP3208 初期化 ----
    if (g_unit_type & UNIT_CURRENT) {
        Serial.println("Init SPI for MCP3208...");
        pinMode(PIN_SPI_CS, OUTPUT);
        digitalWrite(PIN_SPI_CS, HIGH);
        SPI.setSCK(PIN_SPI_SCK);
        SPI.setRX(PIN_SPI_MISO);
        SPI.setTX(PIN_SPI_MOSI);
        SPI.begin();
        g_current_window_start = millis();
        Serial.println("  MCP3208 ready (CH0, VREF=3.3V)");
    }

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

    // D1 ハートビートパルス（5秒周期、Core0+Core1 両方が生きているときのみ）
    uint32_t now = millis();
    if (now - g_lastHbMs >= D1_PERIOD_MS) {
        g_lastHbMs = now;
        uint32_t c1hb;
        bool sensor_err;
        mutex_enter_blocking(&g_mutex);
        c1hb       = g_shared.core1_heartbeat;
        sensor_err = g_shared.sensor_error;
        mutex_exit(&g_mutex);
        bool core1_alive = (c1hb != g_lastCore1Hb);
        g_lastCore1Hb = c1hb;
        if (core1_alive) {
            g_d1FlashMs = now;
            digitalWrite(PIN_LED_D1, HIGH);
        }
        digitalWrite(PIN_LED_D4, sensor_err ? HIGH : LOW);
    }

    // D1 ハートビートパルス消灯
    if (g_d1FlashMs > 0 && (now - g_d1FlashMs >= D1_FLASH_MS)) {
        g_d1FlashMs = 0;
        digitalWrite(PIN_LED_D1, LOW);
    }

    // D2 LoRa受信フラッシュ（100ms後に消灯）
    if (g_d2FlashMs > 0 && (millis() - g_d2FlashMs >= D2_FLASH_MS)) {
        g_d2FlashMs = 0;
        digitalWrite(PIN_LED_D2, LOW);
    }

}

// ===== Core1 =====
// Wire/センサーはCore0のsetup()で初期化済み。Core1はWireを排他的に使用する。
void setup1() {}

void loop1() {
    bool did_heartbeat = false;

    // === パトライト: TSL2561 高速サイクリング（~45ms/3ch） ===
    if (g_unit_type & UNIT_PATLITE) {
        for (uint8_t ch = 0; ch < 3; ch++) {
            if (!g_tsl_ok[ch]) continue;
            sensors_event_t event;
            tcaSelect(ch);
            delay(2);
            g_tsl[ch]->getEvent(&event);
            updatePatliteMax(ch, event.light);
        }
        mutex_enter_blocking(&g_mutex);
        g_shared.patlite_max[0] = g_patlite_local_max[0];
        g_shared.patlite_max[1] = g_patlite_local_max[1];
        g_shared.patlite_max[2] = g_patlite_local_max[2];
        g_shared.core1_heartbeat++;
        mutex_exit(&g_mutex);
        did_heartbeat = true;
    }

    // === 電流: MCP3208 1サンプル/ループ + 100ms RMS計算 ===
    if (g_unit_type & UNIT_CURRENT) {
        uint16_t raw = readMCP3208_raw(ADC_CURRENT_CH);
        int32_t v_ac = (int32_t)raw - ADC_DC_OFFSET;
        g_current_sum    += raw;
        if ((int32_t)raw < g_current_raw_min) g_current_raw_min = raw;
        if ((int32_t)raw > g_current_raw_max) g_current_raw_max = raw;
        g_current_sum_sq += (int64_t)v_ac * v_ac;
        g_current_count++;

        if (millis() - g_current_window_start >= CURRENT_WINDOW_MS) {
            if (g_current_count > 0) {
                float rms_counts  = sqrtf((float)g_current_sum_sq / (float)g_current_count);
                float current_rms = rms_counts * (3.3f / 4096.0f) * 20.0f;   // 2000/100Ω
                float mean_raw    = (float)g_current_sum / (float)g_current_count;
                float dc_err      = mean_raw - (float)ADC_DC_OFFSET;
                // [ADC診断] 100msウィンドウごとに出力
                Serial.printf("[ADC] N=%lu mean=%.1f(%.4fV) min=%d max=%d dc_err=%.1f rms_cnt=%.2f -> %.2fA\n",
                    g_current_count,
                    mean_raw, mean_raw * (3.3f / 4096.0f),
                    g_current_raw_min, g_current_raw_max,
                    dc_err, rms_counts, current_rms);
                mutex_enter_blocking(&g_mutex);
                g_shared.current_rms = current_rms;
                mutex_exit(&g_mutex);
            }
            g_current_sum          = 0;
            g_current_raw_min      = 32767;
            g_current_raw_max      = 0;
            g_current_sum_sq       = 0;
            g_current_count        = 0;
            g_current_window_start = millis();
        }

        if (!did_heartbeat) {
            // 電流ユニット単独: HB更新 + ~1kHz レート制御
            mutex_enter_blocking(&g_mutex);
            g_shared.core1_heartbeat++;
            mutex_exit(&g_mutex);
            did_heartbeat = true;
            delayMicroseconds(900);  // SPI転送~100μs込みで合計~1ms/サンプル
        }
        // 兼務の場合: パトライトブロックでHB更新済み・TSL2561がレート制限(~45ms/cycle)
    }

    // === DIP未設定フォールバック ===
    if (!did_heartbeat) {
        mutex_enter_blocking(&g_mutex);
        g_shared.core1_heartbeat++;
        mutex_exit(&g_mutex);
        delay(50);
    }
}
