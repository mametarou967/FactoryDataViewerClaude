/**
 * edge_unit.ino  –  Phase 2 Step4+5: SSD1306 + ボタンUI + ローカルテストモード
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
 *   - SSD1306 OLED表示（Wire1: GPIO26/27）
 *   - 3ボタンUI（デバウンス付き）: 設定確認・ADDH変更・CH変更
 *   - Step5 ローカルテストモード: パトライト lux 表示 / 電流波形スナップショット + RMS 表示
 *
 * 未実装（後続Step）:
 *   - Phase3/4: GW + Webアプリ統合
 *
 * E220 コンフィグモード(M0=HIGH,M1=HIGH)では UART は常に 9600bps 固定。
 * 通常動作モードでは DESIRED_REG0 に設定したボーレート（9600bps）を使用する。
 */

#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <pico/mutex.h>
#include "hardware/flash.h"
#include "hardware/sync.h"
#include "hardware/watchdog.h"

// ===== UIモード（インクルード直後に定義: Arduino自動プロトタイプ生成との衝突防止） =====
enum UIMode : uint8_t {
    MODE_NORMAL = 0,
    MODE_MENU,
    MODE_CONF_VIEW,
    MODE_EDIT_ADDH,
    MODE_EDIT_CHANNEL,
    MODE_TEST_PATLITE,   // Step5: パトライトテスト（lux値表示）
    MODE_TEST_CURRENT,   // Step5: 電流テスト（波形スナップショット + RMS）
};

// ===== ボタンデバウンス構造体（pollBtnプロトタイプより前に定義必須） =====
struct BtnState { bool prev; uint32_t lastMs; };

// ===== OTA定数 =====
#define OTA_BANK_OFFSET   0x100000UL   // Bank B 物理オフセット（フラッシュ1MB境界）
#define OTA_CHUNK_SIZE    128
#define OTA_MAX_FIRMWARE  (1 * 1024 * 1024)  // 1MB（Bank B サイズ）
// Bank B 末尾のマジック領域: "OTA_READY"(9B) + fw_size(4B,BE) + 3B padding = 16B
#define OTA_MAGIC_OFFSET  (OTA_BANK_OFFSET + OTA_MAX_FIRMWARE - 16)

// ===== OTA状態機械 =====
enum OtaState : uint8_t {
    OTA_IDLE  = 0,
    OTA_INIT,
    OTA_RECV,
    OTA_FIN,
    OTA_DONE,
    OTA_FAIL
};

struct OtaCtx {
    OtaState state           = OTA_IDLE;
    uint32_t total_size      = 0;
    uint32_t total_crc32     = 0;
    uint16_t chunk_size      = OTA_CHUNK_SIZE;
    uint16_t expected_seq    = 0;
    uint32_t written         = 0;
    uint32_t chunks_rcvd     = 0;
    // ページバッファ: CHUNK=128B、PAGE=256B → 2チャンク分貯めてから1ページ書き込む
    uint8_t  page_buf[FLASH_PAGE_SIZE];  // 256B（staticグローバルなのでRAM配置確定）
    uint16_t page_fill       = 0;        // 現ページに積んだバイト数
    uint32_t page_base_off   = 0;        // 現ページのフラッシュ物理オフセット（ページ境界アライン済み）
};
static OtaCtx g_ota;

// ===== Unit Type =====
static const uint8_t UNIT_PATLITE = 0x01;
static const uint8_t UNIT_CURRENT = 0x02;

static uint8_t g_unit_type = 0;

// ===== I2C / TCA9548A =====
static const uint8_t PIN_I2C0_SDA = 20;
static const uint8_t PIN_I2C0_SCL = 21;
static const uint8_t TCA_ADDR     = 0x70;

// ===== I2C1 / SSD1306 =====
static const uint8_t  PIN_I2C1_SDA    = 26;
static const uint8_t  PIN_I2C1_SCL    = 27;
static const uint8_t  OLED_ADDR       = 0x3C;
static const uint8_t  OLED_W          = 128;
static const uint8_t  OLED_H          = 64;
static const uint32_t BTN_DEBOUNCE_MS = 150;

// ===== コア間共有データ =====
struct SharedData {
    float    patlite_max[3];   // TCA CH0/1/2 の直近1.5秒max値 [lux]
    float    current_rms;      // RMS電流値 [A]
    bool     sensor_error;     // センサー異常フラグ
    uint32_t core1_heartbeat;  // Core1生存確認カウンタ
    // Step5: 電流波形スナップショット（テストモード用）
    int16_t  wave_buf[128];    // 128サンプル（~128ms @ 1kHz）の生AC値
    bool     wave_ready;       // true=128サンプル取得完了・描画可能
};
static SharedData g_shared = {};
static mutex_t    g_mutex;

// ===== TSL2561センサー =====
static Adafruit_TSL2561_Unified g_tsl0(TSL2561_ADDR_FLOAT, 1000);
static Adafruit_TSL2561_Unified g_tsl1(TSL2561_ADDR_FLOAT, 1001);
static Adafruit_TSL2561_Unified g_tsl2(TSL2561_ADDR_FLOAT, 1002);
static Adafruit_TSL2561_Unified *g_tsl[3] = {&g_tsl0, &g_tsl1, &g_tsl2};
static volatile bool g_tsl_ok[3]    = {};
static volatile bool g_setup_done   = false;  // Core0のsetup()完了フラグ（Core1がloop1()開始前に待つ）

// ===== 1.5秒maxウィンドウ（Core1ローカル） =====
static const uint32_t PATLITE_WINDOW_MS = 1500;
static float    g_patlite_local_max[3]  = {};
static uint32_t g_patlite_expire_ms[3]  = {};

// ===== MCP3208 電流計測 =====
// 回路: CTL-24-CLS → R1(33Ω) → 計測点 (DCバイアス1.65V) → MCP3208 CH0
// 変換: Io(Arms) = Vac_rms(V) × (2000/33) ≈ 60.6   ※ n/RL = 2000/33, K≈1 (Io≥10A)
// 最大安全電流: ~100A ピーク（ADC入力: 1.65±1.49V, 3.3V以内）
static const uint8_t  PIN_SPI_MISO      = 16;
static const uint8_t  PIN_SPI_CS        = 17;
static const uint8_t  PIN_SPI_SCK       = 18;
static const uint8_t  PIN_SPI_MOSI      = 19;
static const uint8_t  ADC_CURRENT_CH    = 0;      // MCP3208 CH0
static const int32_t  ADC_DC_OFFSET     = 2048;   // 1.65V / 3.3V × 4096
static const uint32_t CURRENT_WINDOW_MS = 100;    // RMSウィンドウ（50Hzで5周期）
// ノイズフロア補正値 [Arms]: 電線未接続(0A)状態で実測した値を設定する。
// 補正式: I_real = sqrt(I_meas^2 - N^2)。0.0fで補正無効。
static const float    CURRENT_NOISE_FLOOR_A = 0.0f;

// 電流計測（Core1ローカル）
static int64_t  g_current_sum_sq       = 0;
static uint32_t g_current_count        = 0;
static uint32_t g_current_window_start = 0;

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
static const uint8_t FW_MINOR = 6;
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

// ===== Serial2 初期化済みフラグ =====
static bool g_serial2_inited = false;

// ===== AUX割り込みフラグ =====
static volatile bool g_auxRise = false;
static void onAuxRise() { g_auxRise = true; }

// ===== Core1 協調的一時停止（flash操作中の安全停止） =====
// multicore_lockout_victim_init() は arduino-pico の SIO_IRQ_PROC1 ハンドラと衝突して
// Core1 を panic させるため使用不可。代わりに volatile フラグで協調停止する。
static volatile bool g_core1_pause_req = false;  // Core0→Core1: 停止要求
static volatile bool g_core1_paused    = false;  // Core1→Core0: 停止確認

// RAM常駐: Core1 は flash 操作中ここでスピン（XIP 無効中でも安全）
void __no_inline_not_in_flash_func(core1ParkInRam)(void) {
    g_core1_paused = true;
    while (g_core1_pause_req) tight_loop_contents();
    g_core1_paused = false;
}

// Core0 用: Core1 が RAM に退避するまで待つ（200ms タイムアウト）
static void requestCore1Pause() {
    g_core1_pause_req = true;
    uint32_t t = millis();
    while (!g_core1_paused && (millis() - t < 200)) tight_loop_contents();
}

// Core0 用: Core1 を再開させる
static void releaseCore1() {
    g_core1_pause_req = false;
}

// ===== SSD1306 OLEDオブジェクト =====
static Adafruit_SSD1306 g_oled(OLED_W, OLED_H, &Wire1, -1);
static bool g_oled_ok   = false;  // OLED初期化成功フラグ（reportError内で参照）
static bool g_hw_error  = false;  // ハードウェアエラーフラグ（reportError()で設定、再起動まで維持）
// エラー内容をOLEDに継続表示するための文字列（updateOLED()で使用）
static char g_hw_err_title[21] = "";
static char g_hw_err_line1[21] = "";

// ===== UI状態 =====
static UIMode g_uiMode = MODE_NORMAL;

// ===== ボタンデバウンスインスタンス =====
static BtnState g_selState  = {true, 0};
static BtnState g_okState   = {true, 0};
static BtnState g_backState = {true, 0};

// ===== 動的メニュー（ユニット種別に応じて buildMenu() で構築） =====
struct MenuItem { const char *label; uint8_t action; };
static MenuItem g_menuItems[6];
static uint8_t  g_menuCount = 0;

// ===== UI状態変数 =====
static int8_t  g_menuCursor    = 0;
static uint8_t g_editValue     = 0;   // 編集中の値
static int8_t  g_confScrollPos = 0;   // ConfView スクロール位置（0〜1）
static uint32_t g_lastOledMs   = 0;

// ===== 状態変数 =====
static uint32_t g_lastHbMs    = 0;
static uint32_t g_d1FlashMs   = 0;  // D1点灯開始時刻（0=消灯中）
static uint32_t g_d2FlashMs   = 0;  // D2点灯開始時刻（0=消灯中）
static uint32_t g_lastCore1Hb = 0;

// ===== Step5: 波形キャプチャ用グローバル =====
static volatile bool g_wave_capture = false;   // Core0→Core1: キャプチャ指示フラグ
// Core1プライベート（loop1からのみアクセス）
static int16_t  s_wave_local[128]   = {};
static bool     s_prev_capture      = false;
static uint16_t s_wave_idx          = 0;

// ===== Serial2 初期化ヘルパー =====
// [注意] Serial2.end() → begin() の繰り返しは irq_set_exclusive_handler() で設定した
//        ハンドラを irq_remove_handler() で除去する際に USB CDC (tud_task) の状態を
//        破壊し、その後の delay() がハングする原因になる。
//        このプロジェクトではボーレートは常に 9600bps 固定のため、
//        初回のみ完全初期化し、2回目以降は RX バッファのクリアのみ行う。
static void serial2Begin(uint32_t baud) {
    if (!g_serial2_inited) {
        // 初回のみ完全初期化（Serial2.end() は呼ばない）
        Serial2.setTX(PIN_LORA_TX);
        Serial2.setRX(PIN_LORA_RX);
        Serial2.setFIFOSize(256);  // OTA DATAパケット(135B)受信のため拡張（デフォルト32Bでは溢れる）
        Serial2.begin(baud);
        g_serial2_inited = true;
    } else {
        // 2回目以降: RX バッファのみクリア（UART 再初期化は行わない）
        while (Serial2.available()) Serial2.read();
    }
    delay(50);
}

// ===== E220 モード切替 =====
// 起動時チェック用: タイムアウト付き（timeout_ms以内にHIGHにならなければfalseを返す）
static bool e220WaitAuxHighTimeout(uint32_t timeout_ms) {
    uint32_t start   = millis();
    uint32_t lastDot = millis();
    while (digitalRead(PIN_LORA_AUX) == LOW) {
        if (millis() - start >= timeout_ms) { Serial.println(); return false; }
        if (millis() - lastDot >= 500) { Serial.print("."); lastDot = millis(); }
        delay(1);
    }
    Serial.println();
    return true;
}

// モード切替用: 無限待ち（E220存在確認済み後に使用）
static void e220WaitAuxHigh() {
    uint32_t lastDot = millis();
    while (digitalRead(PIN_LORA_AUX) == LOW) {
        if (millis() - lastDot >= 500) {
            Serial.print(".");
            lastDot = millis();
        }
        delay(1);
    }
    Serial.println();  // ドット行を改行
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
// USB差し込み時は電源立ち上がりが遅くタイミングによって初期化に失敗することがある。
// 各チャンネルを最大3回リトライして確実に初期化する。
static void initPatliteSensors() {
    bool any_error = false;
    for (uint8_t ch = 0; ch < 3; ch++) {
        bool ok = false;
        for (int retry = 0; retry < 3 && !ok; retry++) {
            tcaSelect(ch);
            delay(10);  // TCAチャンネル切替安定待ち（旧2ms→10ms）
            if (g_tsl[ch]->begin()) {
                ok = true;
            } else if (retry < 2) {
                Serial.printf("  [RETRY %d] TSL2561 CH%d...\n", retry + 1, ch);
                delay(20);
            }
        }
        if (!ok) {
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
    // D4点灯はreportError()経由（g_hw_errorを設定）ではなく、
    // ここでは直接点灯しない。呼び出し元のsetup()でreportError()を呼ぶ。
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

// ===== OTA CRC ユーティリティ =====
static uint16_t crc16_ccitt(const uint8_t *data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

static uint32_t crc32_ieee(const uint8_t *data, uint32_t len) {
    uint32_t crc = 0xFFFFFFFFUL;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc & 1) ? (crc >> 1) ^ 0xEDB88320UL : (crc >> 1);
    }
    return crc ^ 0xFFFFFFFFUL;
}

// ===== RAM常駐バイト操作（applyOTA_impl が Bank A 消去後も安全に呼べるよう） =====
// memcpy/memset は libc (Bank A フラッシュ) に置かれる。Bank A 消去後にそれらを呼ぶと
// XIPキャッシュミス → 0xFF fetch → 不正命令 → HardFault となるため、
// __no_inline_not_in_flash_func で同じく RAM 常駐にした独自版を使う。
static void __no_inline_not_in_flash_func(ram_memcpy)(
        uint8_t *dst, const uint8_t *src, uint32_t n) {
    while (n--) *dst++ = *src++;
}
static void __no_inline_not_in_flash_func(ram_memset)(
        uint8_t *dst, uint8_t v, uint32_t n) {
    while (n--) *dst++ = v;
}

// ===== applyOTA_impl: RAMから実行（flash操作中はXIPアクセス不可） =====
// Bank B (物理オフセット OTA_BANK_OFFSET) → Bank A (物理オフセット 0) にコピーして再起動
//
// 重要: flash_range_program() 実行中は XIP が無効になるため、
//       Bank B の XIP アドレス (XIP_BASE+OTA_BANK_OFFSET) を直接 data 引数に渡すと
//       ハードフォルトする。各ページを flash 操作前にRAMバッファへコピーしてから渡す。
// 重要2: Bank A 消去後は libc の memcpy/memset が呼べないため ram_memcpy/ram_memset を使う。
void __no_inline_not_in_flash_func(applyOTA_impl)(uint32_t fw_size) {
    // ===== STEP -1: Bank A 消去前にウォッチドッグを有効化 =====
    // watchdog_enable() は Bank A (フラッシュ) にある SDK 関数。
    // Bank A 消去後は呼び出せないため、消去前のこのタイミングで必ず呼ぶ。
    // 全操作（消去+コピー ≈ 2秒）が終わった後、LOAD に最小値を書いて即時リセット。
    watchdog_enable(5000, false);   // ← XIP 有効・Bank A 存在中に呼ぶ（タイムアウト 5000ms）

    // 0. Bank B のマジックセクターを最初に消去（再起動ループを防ぐ）
    //    OTA_MAGIC_OFFSET(0x1FFFF0) が含まれるセクター先頭 = 0x1FF000
    uint32_t magic_sector = OTA_MAGIC_OFFSET & ~((uint32_t)FLASH_SECTOR_SIZE - 1);
    flash_range_erase(magic_sector, FLASH_SECTOR_SIZE);

    // 1. Bank A を fw_size 分消去
    //    ※ この後 Bank A の SDK 関数は呼べない。ウォッチドッグは既に有効（STEP-1 で設定済み）。
    uint32_t erase_size = (fw_size + FLASH_SECTOR_SIZE - 1) & ~(FLASH_SECTOR_SIZE - 1);
    flash_range_erase(0, erase_size);

    // 2. Bank B → Bank A コピー
    //    ※ Bank A 消去後なので libc memcpy/memset は使えない → ram_memcpy/ram_memset を使う
    uint8_t ram_page[FLASH_PAGE_SIZE];  // スタック = RAM。この関数自体もRAM実行なので問題なし
    for (uint32_t off = 0; off < fw_size; off += FLASH_PAGE_SIZE) {
        uint32_t plen = fw_size - off;
        if (plen > FLASH_PAGE_SIZE) plen = FLASH_PAGE_SIZE;
        const uint8_t *src = (const uint8_t *)(XIP_BASE + OTA_BANK_OFFSET + off);
        ram_memcpy(ram_page, src, plen);
        if (plen < FLASH_PAGE_SIZE) ram_memset(ram_page + plen, 0xFF, FLASH_PAGE_SIZE - plen);
        flash_range_program(off, ram_page, FLASH_PAGE_SIZE);
    }

    // 3. コピー完了。LOAD に最小値(2)を書いてウォッチドッグを即時発火させる。
    //    STEP-1 で watchdog_enable(5000) により CTRL.ENABLE=1 かつ TICK 動作中。
    //    LOAD に書くと COUNT が即座に上書きされ、次のティック(≈1µs)でリセット発火。
    //    WATCHDOG_BASE=0x40058000, LOAD offset=0x004
    *((volatile uint32_t*)(0x40058004u)) = 2u;
    while (true) tight_loop_contents();  // WDT発火待ち（ここには到達しない）
}

// ===== OTA: Bank B末尾マジック書き込み =====
// フォーマット: "OTA_READY"(9B) + fw_size(4B,BE) + 3B padding
// OTA_MAGIC_OFFSET は OTA_BANK_OFFSET+OTA_MAX_FIRMWARE-16 = 0x1FFFF0（ページ境界ではない）。
// flash_range_program はページ境界アライン必須なので、OTA_MAGIC_OFFSET を含む
// ページ先頭 (page_off) から書く。ページ内オフセット (intra) を計算してそこへマジックを配置する。
static void otaWriteMagic(uint32_t fw_size) {
    uint32_t page_off = OTA_MAGIC_OFFSET & ~(uint32_t)(FLASH_PAGE_SIZE - 1);
    uint32_t intra    = OTA_MAGIC_OFFSET - page_off;  // ページ内オフセット (= 0xF0 = 240)

    uint8_t magic_buf[FLASH_PAGE_SIZE];
    memset(magic_buf, 0xFF, FLASH_PAGE_SIZE);  // 消去済み状態(0xFF)で初期化
    uint8_t *p = magic_buf + intra;
    p[0] = 'O'; p[1] = 'T'; p[2] = 'A';
    p[3] = '_'; p[4] = 'R'; p[5] = 'E';
    p[6] = 'A'; p[7] = 'D'; p[8] = 'Y';
    p[9]  = (fw_size >> 24) & 0xFF;
    p[10] = (fw_size >> 16) & 0xFF;
    p[11] = (fw_size >>  8) & 0xFF;
    p[12] = (fw_size >>  0) & 0xFF;

    requestCore1Pause();
    uint32_t ints = save_and_disable_interrupts();
    flash_range_program(page_off, magic_buf, FLASH_PAGE_SIZE);
    restore_interrupts(ints);
    releaseCore1();
}

// ===== OTA: Bank B マジック消去 =====
static void otaClearMagic() {
    uint32_t page_off = OTA_MAGIC_OFFSET & ~(FLASH_SECTOR_SIZE - 1);
    requestCore1Pause();
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(page_off, FLASH_SECTOR_SIZE);
    restore_interrupts(ints);
    releaseCore1();
}

// ===== OTA ハンドラ =====

// UI → GW: READY レスポンス
// [ADDR_H][ADDR_L]['U']['R'][chunk_size_H][chunk_size_L][CR][LF]  8B
static void sendOtaReady() {
    uint16_t cs = OTA_CHUNK_SIZE;
    uint8_t resp[] = {
        g_e220.addH, g_e220.addL,
        'U', 'R',
        (uint8_t)(cs >> 8), (uint8_t)(cs & 0xFF),
        '\r', '\n'
    };
    sendToGW(resp, sizeof(resp));
}

// UK: ACK
// [ADDR_H][ADDR_L]['U']['K'][seq_H][seq_L][next_seq_H][next_seq_L][CR][LF]  10B
static void sendOtaAck(uint16_t seq, uint16_t next_seq) {
    uint8_t resp[] = {
        g_e220.addH, g_e220.addL,
        'U', 'K',
        (uint8_t)(seq >> 8), (uint8_t)(seq & 0xFF),
        (uint8_t)(next_seq >> 8), (uint8_t)(next_seq & 0xFF),
        '\r', '\n'
    };
    sendToGW(resp, sizeof(resp));
}

// UN: NACK
// [ADDR_H][ADDR_L]['U']['N'][seq_H][seq_L][err][CR][LF]  9B
static void sendOtaNack(uint16_t seq, uint8_t err) {
    uint8_t resp[] = {
        g_e220.addH, g_e220.addL,
        'U', 'N',
        (uint8_t)(seq >> 8), (uint8_t)(seq & 0xFF),
        err,
        '\r', '\n'
    };
    sendToGW(resp, sizeof(resp));
}

// UD: DONE（FIN後のCRC確認完了）
// [ADDR_H][ADDR_L]['U']['D'][crc32(4B,BE)][CR][LF]  10B
static void sendOtaDone(uint32_t crc32) {
    uint8_t resp[] = {
        g_e220.addH, g_e220.addL,
        'U', 'D',
        (uint8_t)(crc32 >> 24), (uint8_t)(crc32 >> 16),
        (uint8_t)(crc32 >>  8), (uint8_t)(crc32 &  0xFF),
        '\r', '\n'
    };
    sendToGW(resp, sizeof(resp));
}

// UF: FAIL
// [ADDR_H][ADDR_L]['U']['F'][code][reason_H][reason_L][CR][LF]  9B
static void sendOtaFail(uint8_t code, uint16_t reason) {
    uint8_t resp[] = {
        g_e220.addH, g_e220.addL,
        'U', 'F',
        code,
        (uint8_t)(reason >> 8), (uint8_t)(reason & 0xFF),
        '\r', '\n'
    };
    sendToGW(resp, sizeof(resp));
}

// INIT: UI(2B) + total_size(4B,BE) + total_crc32(4B,BE) + chunk_size(2B,BE) + reserved(1B) = 13B
static void handleOtaInit(const uint8_t *buf, int len) {
    if (len < 13) { sendOtaFail(0x01, len); return; }
    uint32_t total_size  = ((uint32_t)buf[2] << 24) | ((uint32_t)buf[3] << 16)
                         | ((uint32_t)buf[4] <<  8) | buf[5];
    uint32_t total_crc32 = ((uint32_t)buf[6] << 24) | ((uint32_t)buf[7] << 16)
                         | ((uint32_t)buf[8] <<  8) | buf[9];
    // uint16_t chunk_size  = ((uint16_t)buf[10] << 8) | buf[11];  // 将来拡張用

    if (total_size == 0 || total_size > OTA_MAX_FIRMWARE - 16) {
        sendOtaFail(0x02, 0); return;
    }

    Serial.printf("[OTA] INIT total=%lu crc32=0x%08lX\n", total_size, total_crc32);

    // Bank B 全体を消去（Core1停止中に実行）
    requestCore1Pause();
    uint32_t ints = save_and_disable_interrupts();
    flash_range_erase(OTA_BANK_OFFSET, OTA_MAX_FIRMWARE);
    restore_interrupts(ints);
    releaseCore1();

    g_ota.state          = OTA_RECV;
    g_ota.total_size     = total_size;
    g_ota.total_crc32    = total_crc32;
    g_ota.chunk_size     = OTA_CHUNK_SIZE;
    g_ota.expected_seq   = 0;
    g_ota.written        = 0;
    g_ota.chunks_rcvd    = 0;
    g_ota.page_fill      = 0;
    g_ota.page_base_off  = 0;
    memset(g_ota.page_buf, 0xFF, FLASH_PAGE_SIZE);

    sendOtaReady();
    onRxSuccess();
    Serial.println("[OTA] Bank B erased. READY sent.");
}

// DATA: UD(2B) + seq(2B,BE) + chunk_crc16(2B,BE) + len(1B) + data(len B)
static void handleOtaData(const uint8_t *buf, int pktlen) {
    if (g_ota.state != OTA_RECV) {
        sendOtaNack(0, 0x10); return;
    }
    if (pktlen < 7) { sendOtaNack(0, 0x11); return; }

    uint16_t seq      = ((uint16_t)buf[2] << 8) | buf[3];
    uint16_t rcrc16   = ((uint16_t)buf[4] << 8) | buf[5];
    uint8_t  dlen     = buf[6];

    Serial.printf("[OTA-D] pktlen=%d dlen=%d (need=%d)\n", pktlen, (int)dlen, 7+(int)dlen);
    if (pktlen < 7 + (int)dlen) { sendOtaNack(seq, 0x12); return; }

    const uint8_t *data = buf + 7;

    // seq チェック（重複は ACK 返して無視、先行はエラー）
    if (seq != g_ota.expected_seq) {
        Serial.printf("[OTA] seq mismatch: got %d, expect %d\n", seq, g_ota.expected_seq);
        sendOtaNack(seq, 0x20);
        return;
    }

    // CRC16 検証
    uint16_t calc_crc = crc16_ccitt(data, dlen);
    if (calc_crc != rcrc16) {
        Serial.printf("[OTA] CRC16 mismatch seq=%d: got 0x%04X calc 0x%04X\n",
                      seq, rcrc16, calc_crc);
        sendOtaNack(seq, 0x30);
        return;
    }

    // ページバッファに積む
    // CHUNK=128B, PAGE=256B → 2チャンクで1ページ。ページ境界をまたがないよう管理する。
    uint32_t seq_off  = OTA_BANK_OFFSET + (uint32_t)seq * OTA_CHUNK_SIZE;
    uint32_t page_off = seq_off & ~(uint32_t)(FLASH_PAGE_SIZE - 1);  // 256B境界切り捨て
    uint16_t intra    = (uint16_t)(seq_off - page_off);               // ページ内オフセット（0 or 128）

    if (intra == 0) {
        // 新しいページの先頭チャンク: バッファを0xFF（消去済み）でリセット
        memset(g_ota.page_buf, 0xFF, FLASH_PAGE_SIZE);
        g_ota.page_base_off = page_off;
        g_ota.page_fill     = 0;
    }
    memcpy(g_ota.page_buf + intra, data, dlen);
    g_ota.page_fill += dlen;

    // ページが満杯 or 最終チャンク → フラッシュに書き込む
    bool last_chunk = ((g_ota.written + dlen) >= g_ota.total_size);
    if (g_ota.page_fill >= FLASH_PAGE_SIZE || last_chunk) {
        // 最終チャンクでページ未満の場合、残りを 0xFF（消去済みと同値）で明示パディング
        if (g_ota.page_fill < FLASH_PAGE_SIZE) {
            memset(g_ota.page_buf + g_ota.page_fill, 0xFF,
                   FLASH_PAGE_SIZE - g_ota.page_fill);
        }
        requestCore1Pause();
        uint32_t ints = save_and_disable_interrupts();
        flash_range_program(g_ota.page_base_off, g_ota.page_buf, FLASH_PAGE_SIZE);
        restore_interrupts(ints);
        releaseCore1();
        g_ota.page_fill = 0;
    }

    g_ota.written     += dlen;
    g_ota.chunks_rcvd++;
    g_ota.expected_seq = seq + 1;

    Serial.printf("[OTA] DATA seq=%d len=%d written=%lu\n", seq, dlen, g_ota.written);
    sendOtaAck(seq, g_ota.expected_seq);
    onRxSuccess();
}

// FIN: UF(2B) + total_size(4B,BE)
static void handleOtaFin(const uint8_t *buf, int len) {
    if (g_ota.state != OTA_RECV) {
        sendOtaFail(0x40, 0); return;
    }
    if (len < 6) { sendOtaFail(0x41, len); return; }

    uint32_t fin_size = ((uint32_t)buf[2] << 24) | ((uint32_t)buf[3] << 16)
                      | ((uint32_t)buf[4] <<  8) | buf[5];

    if (fin_size != g_ota.total_size) {
        sendOtaFail(0x42, 0); return;
    }
    if (g_ota.written < g_ota.total_size) {
        Serial.printf("[OTA] FIN: written=%lu < total=%lu\n",
                      g_ota.written, g_ota.total_size);
        sendOtaFail(0x43, 0); return;
    }

    // Bank B の CRC32 計算
    const uint8_t *bank_b = (const uint8_t *)(XIP_BASE + OTA_BANK_OFFSET);
    uint32_t calc_crc32 = crc32_ieee(bank_b, g_ota.total_size);

    if (calc_crc32 != g_ota.total_crc32) {
        Serial.printf("[OTA] CRC32 mismatch: got 0x%08lX calc 0x%08lX\n",
                      g_ota.total_crc32, calc_crc32);
        g_ota.state = OTA_FAIL;
        sendOtaFail(0x44, 0);
        return;
    }

    // マジックワードをBank B末尾に書き込んでOTA適用フラグ立て
    otaWriteMagic(g_ota.total_size);
    g_ota.state = OTA_DONE;

    Serial.printf("[OTA] FIN OK. crc32=0x%08lX. Rebooting...\n", calc_crc32);
    sendOtaDone(calc_crc32);
    onRxSuccess();

    // 少し待ってから再起動（GWがDONEを受け取れるように）
    delay(500);
    requestCore1Pause();  // Core1 を RAM に退避させてから flash 操作
    uint32_t ints = save_and_disable_interrupts();
    applyOTA_impl(g_ota.total_size);
    // ここには戻らない
}

// ABORT: UA(2B) + code(1B) + padding(1B)
static void handleOtaAbort(const uint8_t *buf, int len) {
    (void)buf; (void)len;
    Serial.println("[OTA] ABORT received. Clearing magic and resetting OTA state.");
    if (g_ota.state != OTA_IDLE) {
        otaClearMagic();
    }
    g_ota = OtaCtx{};  // リセット
    // ABORTへの応答は不要
    onRxSuccess();
}

// ===== コマンド受信・ディスパッチ =====
static void processCommand() {
    uint8_t  buf[200];   // OTA DATAパケット(7+128=135B) + 余裕
    int      len = 0;
    uint32_t t   = millis();

    // OTA DATAパケット(UD...)はCRLF終端なし・最大135B
    // 通常パケットはCRLF終端あり・最大32B
    // 受信判定:
    //   OTA DATA ('U','D'): dlenバイト受信完了で終了（CRLFチェックは行わない）
    //   通常パケット: CRLF検出で終了
    // ※ファームウェアバイナリには 0x0D 0x0A が任意の位置に現れるため、
    //   OTA DATAパケット受信中はCRLF誤検出を避ける必要がある
    uint32_t timeout   = 100;
    bool     is_ota_data = false;
    while (len < (int)sizeof(buf)) {
        if (Serial2.available()) {
            uint8_t b = Serial2.read();
            buf[len++] = b;
            // 先頭2バイト確定後にOTA DATAパケットを識別してタイムアウト延長
            if (len == 2 && buf[0] == 'U' && buf[1] == 'D') {
                timeout      = 250;  // OTA DATAパケット用に延長
                is_ota_data  = true;
            }
            if (is_ota_data) {
                // OTA DATA: dlenバイト受信完了で終了（CRLFチェック禁止）
                if (len >= 7) {
                    uint8_t dlen = buf[6];
                    if (len >= 7 + (int)dlen) break;
                }
            } else {
                // 通常パケット: CRLF終端で終了
                if (len >= 3 && buf[len - 2] == '\r' && buf[len - 1] == '\n') break;
            }
        }
        if (millis() - t > timeout) break;
    }

    if (len < 2) return;

    hexDump("[RX]", buf, len);
    uint8_t cmd = buf[0];
    Serial.printf("[RX] cmd='%c'\n", (char)cmd);

    switch (cmd) {
        case 'K': cmdPing();               break;
        case 'V': cmdVersion();            break;
        case 'H': cmdHwInfo();             break;
        case 'P': cmdPatlite();            break;
        case 'C': cmdCurrent();            break;
        case 'U': {
            if (len < 2) { cmdError(ERR_UNKNOWN_CMD); return; }
            char sub = (char)buf[1];
            if      (sub == 'I') handleOtaInit(buf, len);
            else if (sub == 'D') handleOtaData(buf, len);
            else if (sub == 'F') handleOtaFin(buf, len);
            else if (sub == 'A') handleOtaAbort(buf, len);
            else                 cmdError(ERR_UNKNOWN_CMD);
            break;
        }
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

// ===== ボタンデバウンス: 押下(HIGH→LOW)の立下りエッジ検出 =====
// 押下・離す問わず信号変化のたびにタイマーをリセットする。
// これにより、ボタンを長押し後のリリースチャタリングによる誤検知を防ぐ。
static bool pollBtn(uint8_t pin, BtnState &s) {
    bool cur = digitalRead(pin);
    if (cur == s.prev) return false;   // 変化なし
    uint32_t now = millis();
    s.prev = cur;
    if (now - s.lastMs < BTN_DEBOUNCE_MS) { s.lastMs = now; return false; }
    s.lastMs = now;
    return !cur;   // LOWになった（押下）のみTRUE
}

// ===== 通常モードへ戻る（Serial2バッファクリア付き） =====
static void exitToNormal() {
    g_uiMode = MODE_NORMAL;
    while (Serial2.available()) Serial2.read();  // 旧コマンドを破棄
}

// ===== メニュー構築（ユニット種別に応じた項目のみ表示） =====
// action番号: 0=ConfView 1=ADDH 2=Chan 3=PatliteTest 4=CurrentTest 5=Exit
static void buildMenu() {
    g_menuCount = 0;
    g_menuItems[g_menuCount++] = {"Conf view",   0};
    g_menuItems[g_menuCount++] = {"Edit ADDH",   1};
    g_menuItems[g_menuCount++] = {"Edit Chan",   2};
    if (g_unit_type & UNIT_PATLITE) g_menuItems[g_menuCount++] = {"Patlite tst", 3};
    if (g_unit_type & UNIT_CURRENT) g_menuItems[g_menuCount++] = {"Current tst", 4};
    g_menuItems[g_menuCount++] = {"Exit",         5};
}

// ===== UIハンドラ =====
static void handleUI(bool selPr, bool okPr, bool backPr) {
    switch (g_uiMode) {
        case MODE_NORMAL:
            if (okPr) { g_menuCursor = 0; g_uiMode = MODE_MENU; }
            break;

        case MODE_MENU:
            if (selPr) g_menuCursor = (g_menuCursor + 1) % g_menuCount;
            if (okPr) {
                switch (g_menuItems[g_menuCursor].action) {
                    case 0: g_confScrollPos = 0; g_uiMode = MODE_CONF_VIEW; break;
                    case 1: g_editValue = g_e220.addH;    g_uiMode = MODE_EDIT_ADDH;    break;
                    case 2: g_editValue = g_e220.channel; g_uiMode = MODE_EDIT_CHANNEL; break;
                    case 3: g_uiMode = MODE_TEST_PATLITE; break;
                    case 4:
                        mutex_enter_blocking(&g_mutex);
                        g_shared.wave_ready = false;
                        mutex_exit(&g_mutex);
                        g_wave_capture = true;
                        g_uiMode = MODE_TEST_CURRENT;
                        break;
                    case 5: exitToNormal(); break;
                }
            }
            if (backPr) exitToNormal();
            break;

        case MODE_CONF_VIEW:
            // SEL: スクロール（0→1→0 循環）
            if (selPr) g_confScrollPos = (g_confScrollPos + 1) % 3;
            if (backPr) { g_confScrollPos = 0; g_uiMode = MODE_MENU; }
            break;

        case MODE_EDIT_ADDH:
            if (selPr)  g_editValue = (g_editValue + 1) & 0xFF;  // +1（0→255→0ラップ）
            if (backPr) g_editValue = (g_editValue - 1) & 0xFF;  // -1（0→255→0ラップ）
            if (okPr) {
                if (g_editValue != g_e220.addH) {  // 変更ありのみ書き込み
                    E220Config newCfg = g_e220;
                    newCfg.addH = g_editValue;
                    e220WriteConfig(newCfg);
                    g_e220    = newCfg;
                    g_auxRise = false;
                    Serial.printf("[UI] ADDH saved: 0x%02X\n", g_e220.addH);
                }
                g_uiMode = MODE_MENU;
            }
            break;

        case MODE_EDIT_CHANNEL:
            if (selPr)  g_editValue = (g_editValue + 1) % 32;                    // +1（CH 0-31）
            if (backPr) g_editValue = (g_editValue == 0) ? 31 : g_editValue - 1; // -1（0→31ラップ）
            if (okPr) {
                if (g_editValue != g_e220.channel) {  // 変更ありのみ書き込み
                    E220Config newCfg = g_e220;
                    newCfg.channel = g_editValue;
                    e220WriteConfig(newCfg);
                    g_e220    = newCfg;
                    g_auxRise = false;
                    Serial.printf("[UI] Channel saved: %d\n", g_e220.channel);
                }
                g_uiMode = MODE_MENU;
            }
            break;

        case MODE_TEST_PATLITE:
            if (backPr) g_uiMode = MODE_MENU;
            break;

        case MODE_TEST_CURRENT:
            if (backPr) {
                g_wave_capture = false;
                g_uiMode = MODE_MENU;
            }
            break;
    }
}

// ===== 波形描画ヘルパー（Step5: 電流テストモード用） =====
// buf: 128サンプルのAC値（中心0、範囲±ADC_DC_OFFSET）
// y_top/height: OLED上の描画領域
static void drawWaveform(const int16_t *buf, int n, int y_top, int height) {
    int center = y_top + height / 2;
    int half   = height / 2;
    for (int x = 0; x < n && x < OLED_W; x++) {
        int y = center - (int32_t)buf[x] * half / ADC_DC_OFFSET;
        if (y < y_top)           y = y_top;
        if (y >= y_top + height) y = y_top + height - 1;
        g_oled.drawPixel(x, y, SSD1306_WHITE);
    }
}

// ===== OLED描画（200ms間隔） =====
static void updateOLED() {
    uint32_t now = millis();
    if (now - g_lastOledMs < 200) return;
    g_lastOledMs = now;

    g_oled.clearDisplay();
    g_oled.setTextSize(1);
    g_oled.setTextColor(SSD1306_WHITE);

    char buf[24];

    // HWエラー中（E220未接続・DIPエラー等）は通常画面に戻らずエラー表示を維持
    // メニュー操作中はメニュー表示を優先（ユーザーが意図的に操作している）
    if (g_hw_error && g_uiMode == MODE_NORMAL) {
        g_oled.setCursor(0,  0); g_oled.print(g_hw_err_title);
        g_oled.setCursor(0, 16); g_oled.print(g_hw_err_line1);
        g_oled.setCursor(0, 48); g_oled.print("D4:red=HW error");
        g_oled.setCursor(0, 56); g_oled.print("-> reboot needed");
        g_oled.display();
        return;
    }

    switch (g_uiMode) {
        case MODE_NORMAL:
            // ラベル(size1)+値(size2) 分離レイアウト（128px幅に収める）
            // "addr:0x0101"はsize2で11文字×12px=132px>128pxになるため分割
            // y=0-7:   "Addr:"       size1
            // y=8-23:  "0xMMTT"      size2  6chars×12px=72px
            // y=24-31: "CH:"         size1
            // y=32-47: "<ch>"        size2  最大2chars×12px=24px
            // y=48-55: unit+ver      size1
            // y=56-63: "[OK]Menu"    size1
            g_oled.setTextSize(1);
            g_oled.setCursor(0, 0);  g_oled.print("Addr:");
            g_oled.setTextSize(2);
            snprintf(buf, sizeof(buf), "0x%02X%02X", g_e220.addH, g_e220.addL);
            g_oled.setCursor(0, 8);  g_oled.print(buf);
            g_oled.setTextSize(1);
            g_oled.setCursor(0, 24); g_oled.print("CH:");
            g_oled.setTextSize(2);
            snprintf(buf, sizeof(buf), "%d", g_e220.channel);
            g_oled.setCursor(0, 32); g_oled.print(buf);
            g_oled.setTextSize(1);
            g_oled.setCursor(0, 48);
            if ((g_unit_type & (UNIT_PATLITE | UNIT_CURRENT)) == (UNIT_PATLITE | UNIT_CURRENT))
                g_oled.print("pat+cur");
            else if (g_unit_type & UNIT_PATLITE)
                g_oled.print("patlite");
            else if (g_unit_type & UNIT_CURRENT)
                g_oled.print("current");
            else
                g_oled.print("none");
            snprintf(buf, sizeof(buf), "v%d.%d.%d", FW_MAJOR, FW_MINOR, FW_PATCH);
            g_oled.setCursor(60, 48); g_oled.print(buf);
            g_oled.setCursor(0, 56); g_oled.print("[OK]Menu");
            break;

        case MODE_MENU: {
            for (int i = 0; i < g_menuCount; i++) {
                snprintf(buf, sizeof(buf), "%c%s",
                         (g_menuCursor == i) ? '>' : ' ', g_menuItems[i].label);
                g_oled.setCursor(0, i * 8);
                g_oled.print(buf);
            }
            break;
        }

        case MODE_CONF_VIEW: {
            // ---- レジスタデコード ----
            static const char *CV_BAUD[] = {"1200","2400","4800","9600","19200","38400","57600","115200"};
            static const char *CV_PKT[]  = {"200","128","64","32"};
            static const char *CV_BW[]   = {"BW125k","BW250k","BW500k","??"};
            static const char *CV_WOR[]  = {"500","1000","1500","2000","2500","3000","??","??"};
            static const int8_t CV_PWR[] = {-99,13,7,0,1,2,3,4,5,6,7,8,9,10,11,12};
            static const float  CV_BASE[]= {920.6f, 920.7f, 920.8f};
            static const uint32_t CV_BW_HZ[] = {125000, 250000, 500000, 0};

            uint8_t baudBits  = (g_e220.reg0 >> 5) & 0x07;
            uint8_t airBits   =  g_e220.reg0 & 0x1F;
            uint8_t bwIdx     =  airBits & 0x03;
            uint8_t sf        = ((airBits >> 2) & 0x07) + 5;
            uint8_t pktBits   = (g_e220.reg1 >> 6) & 0x03;
            bool    rssiNoise = (g_e220.reg1 >> 5) & 0x01;
            uint8_t pwrBits   =  g_e220.reg1 & 0x0F;
            bool    rssiApp   = (g_e220.reg3 >> 7) & 0x01;
            bool    fixedMode = (g_e220.reg3 >> 6) & 0x01;
            uint8_t worBits   =  g_e220.reg3 & 0x07;
            float   freq      = (bwIdx < 3) ? CV_BASE[bwIdx] + g_e220.channel * 0.2f : 0.0f;
            uint32_t rbInt    = (bwIdx < 3 && sf >= 5 && sf <= 12)
                                ? (uint32_t)((float)sf * CV_BW_HZ[bwIdx] / (1UL << sf) * 0.8f) : 0;

            // ---- 9行コンテンツ生成 ----
            char lines[9][22];
            snprintf(lines[0], 22, "addr:0x%02X%02X", g_e220.addH, g_e220.addL);
            snprintf(lines[1], 22, "CH:%d %.1fMHz", g_e220.channel, freq);
            snprintf(lines[2], 22, "UART:%sbps", CV_BAUD[baudBits < 8 ? baudBits : 0]);
            snprintf(lines[3], 22, "Air:%ubps", (unsigned)rbInt);
            snprintf(lines[4], 22, "SF%d/%s", sf, CV_BW[bwIdx < 3 ? bwIdx : 3]);
            snprintf(lines[5], 22, "Pkt:%sB Pwr:%ddBm",
                     CV_PKT[pktBits < 4 ? pktBits : 0], CV_PWR[pwrBits < 16 ? pwrBits : 0]);
            snprintf(lines[6], 22, "Rn:%s Ra:%s",
                     rssiNoise ? "on" : "off", rssiApp ? "on" : "off");
            snprintf(lines[7], 22, "Mode:%s WOR:%sms",
                     fixedMode ? "Fixed" : "Trans", CV_WOR[worBits < 8 ? worBits : 6]);
            snprintf(lines[8], 22, "R0:%02X R1:%02X R3:%02X",
                     g_e220.reg0, g_e220.reg1, g_e220.reg3);

            // ---- 7行表示（スクロール位置から）+ ヒント ----
            int8_t sp = (g_confScrollPos > 2) ? 2 : g_confScrollPos;
            for (int i = 0; i < 7; i++) {
                g_oled.setCursor(0, i * 8);
                g_oled.print(lines[sp + i]);
            }
            g_oled.setCursor(0, 56);
            g_oled.print("[SEL]scrl [BK]back");
            break;
        }

        case MODE_EDIT_ADDH:
            g_oled.setCursor(0, 0);  g_oled.print("--Edit ADDH--");
            snprintf(buf, sizeof(buf), "now: 0x%02X", g_e220.addH);
            g_oled.setCursor(0, 16); g_oled.print(buf);
            snprintf(buf, sizeof(buf), "new: 0x%02X", g_editValue);
            g_oled.setCursor(0, 24); g_oled.print(buf);
            g_oled.setCursor(0, 40); g_oled.print("[SEL]+1 [BK]-1");
            g_oled.setCursor(0, 48); g_oled.print("[OK]save");
            break;

        case MODE_EDIT_CHANNEL:
            g_oled.setCursor(0, 0);  g_oled.print("--Edit Chan--");
            snprintf(buf, sizeof(buf), "now: %d", g_e220.channel);
            g_oled.setCursor(0, 16); g_oled.print(buf);
            snprintf(buf, sizeof(buf), "new: %d", g_editValue);
            g_oled.setCursor(0, 24); g_oled.print(buf);
            g_oled.setCursor(0, 40); g_oled.print("[SEL]+1 [BK]-1");
            g_oled.setCursor(0, 48); g_oled.print("[OK]save");
            break;

        case MODE_TEST_PATLITE: {
            // パトライトテスト: 3chのlux値をリアルタイム表示
            float lux[3];
            mutex_enter_blocking(&g_mutex);
            lux[0] = g_shared.patlite_max[0];
            lux[1] = g_shared.patlite_max[1];
            lux[2] = g_shared.patlite_max[2];
            mutex_exit(&g_mutex);
            g_oled.setCursor(0, 0);  g_oled.print("--Patlite tst--");
            snprintf(buf, sizeof(buf), "RED: %.1flux", lux[0]);
            g_oled.setCursor(0, 16); g_oled.print(buf);
            snprintf(buf, sizeof(buf), "YEL: %.1flux", lux[1]);
            g_oled.setCursor(0, 24); g_oled.print(buf);
            snprintf(buf, sizeof(buf), "GRN: %.1flux", lux[2]);
            g_oled.setCursor(0, 32); g_oled.print(buf);
            // tsl:YYY = 各TSL2561センサーの初期化状態（Y=正常 N=異常）
            snprintf(buf, sizeof(buf), "tsl:%c%c%c",
                     g_tsl_ok[0]?'Y':'N', g_tsl_ok[1]?'Y':'N', g_tsl_ok[2]?'Y':'N');
            g_oled.setCursor(0, 40); g_oled.print(buf);
            g_oled.setCursor(0, 56); g_oled.print("[BK]exit");
            break;
        }

        case MODE_TEST_CURRENT: {
            // 電流テスト: 波形スナップショット（上48px）+ RMS値 + 操作ヒント
            int16_t wave[128];
            bool    ready;
            float   rms;
            mutex_enter_blocking(&g_mutex);
            ready = g_shared.wave_ready;
            if (ready) memcpy(wave, g_shared.wave_buf, sizeof(wave));
            rms = g_shared.current_rms;
            mutex_exit(&g_mutex);

            if (ready) {
                drawWaveform(wave, 128, 0, 48);
            } else {
                g_oled.setCursor(0, 16); g_oled.print("Capturing...");
            }
            snprintf(buf, sizeof(buf), "RMS:%.2fA", rms);
            g_oled.setCursor(0, 48); g_oled.print(buf);
            g_oled.setCursor(0, 56); g_oled.print("[BK]exit");
            break;
        }
    }

    g_oled.display();
}

// ===== エラー報告（3系統同時出力） =====
// 優先度1: OLED不可 → Serial + D4のみ
// 優先度2: DIP両OFF → Serial + D4 + OLED + halt
// 優先度3: TSL2561失敗 → Serial + D4 + OLED（3秒表示後続行）
// g_oled_ok=false の場合はOLED表示をスキップ
// fatal=true の場合は表示後にhalt（再起動が必要）
static void reportError(bool fatal, const char *serial_msg,
                        const char *oled_title,
                        const char *oled_l1,
                        const char *oled_l2,
                        const char *oled_l3) {
    // 1. シリアル出力
    Serial.printf("[%s] %s\n", fatal ? "FATAL" : "ERROR", serial_msg);

    // 2. D4(赤)点灯 + hw_errorフラグ設定
    // （loop()のハートビートブロックがsensor_errのみで判定するため、
    //   別フラグで保持しないと5秒後に消灯されてしまう）
    g_hw_error = true;
    digitalWrite(PIN_LED_D4, HIGH);
    // updateOLED()でエラー表示を継続するためにメッセージを保存
    if (oled_title) strncpy(g_hw_err_title, oled_title, sizeof(g_hw_err_title) - 1);
    else            strncpy(g_hw_err_title, "!! HW ERROR !!",  sizeof(g_hw_err_title) - 1);
    if (oled_l1)    strncpy(g_hw_err_line1, oled_l1,   sizeof(g_hw_err_line1) - 1);

    // 3. OLED表示（初期化成功時のみ）
    if (g_oled_ok) {
        g_oled.clearDisplay();
        g_oled.setTextSize(1);
        g_oled.setTextColor(SSD1306_WHITE);
        g_oled.setCursor(0,  0); g_oled.print(oled_title);
        if (oled_l1) { g_oled.setCursor(0, 16); g_oled.print(oled_l1); }
        if (oled_l2) { g_oled.setCursor(0, 24); g_oled.print(oled_l2); }
        if (oled_l3) { g_oled.setCursor(0, 32); g_oled.print(oled_l3); }
        g_oled.setCursor(0, 56);
        g_oled.print(fatal ? "-> reboot" : "continuing...");
        g_oled.display();
        if (!fatal) delay(3000);  // 非致命的: 3秒表示してから続行
    }

    if (fatal) while (true) delay(1000);  // 致命的: halt
}

// ===== Setup =====
void setup() {
    // ---- mutex は最初に初期化（Core1起動前に必須） ----
    mutex_init(&g_mutex);

    // ---- OTA適用チェック（Core1起動前・Serial初期化前に実行） ----
    // Bank B末尾にマジック"OTA_READY"があればapplyOTA_implを呼び出す
    // (Core1はまだ起動していないためmulticore_lockout不要)
    {
        const uint8_t *magic = (const uint8_t *)(XIP_BASE + OTA_MAGIC_OFFSET);
        if (magic[0] == 'O' && magic[1] == 'T' && magic[2] == 'A' &&
            magic[3] == '_' && magic[4] == 'R' && magic[5] == 'E' &&
            magic[6] == 'A' && magic[7] == 'D' && magic[8] == 'Y') {
            uint32_t fw_size = ((uint32_t)magic[9]  << 24) | ((uint32_t)magic[10] << 16)
                             | ((uint32_t)magic[11] <<  8) | magic[12];
            // Serial未初期化のためログ出力不可。即座に適用。
            uint32_t ints = save_and_disable_interrupts();
            applyOTA_impl(fw_size);
            // ここには戻らない
        }
    }

    Serial.begin(115200);
    uint32_t t0 = millis();
    while (!Serial && millis() - t0 < 5000) delay(10);
    Serial.printf("=== edge_unit v%d.%d.%d booting ===\n", FW_MAJOR, FW_MINOR, FW_PATCH);

    // ---- LED 初期化 ----
    pinMode(PIN_LED_D1, OUTPUT); digitalWrite(PIN_LED_D1, LOW);
    pinMode(PIN_LED_D2, OUTPUT); digitalWrite(PIN_LED_D2, LOW);
    pinMode(PIN_LED_D3, OUTPUT); digitalWrite(PIN_LED_D3, LOW);
    pinMode(PIN_LED_D4, OUTPUT); digitalWrite(PIN_LED_D4, LOW);

    // ---- [優先度1] SSD1306 早期初期化（DIPエラー表示のためLED init直後に行う） ----
    Wire1.setSDA(PIN_I2C1_SDA);
    Wire1.setSCL(PIN_I2C1_SCL);
    Wire1.begin();
    // I2Cアドレススキャンで物理的な接続を確認（beginTransmission+endTransmissionでACK/NACKを検出）
    // endTransmission()戻り値: 0=ACK(接続あり), 2=NACK(接続なし/アドレス不一致)
    Wire1.beginTransmission(OLED_ADDR);
    uint8_t i2c_err = Wire1.endTransmission();
    if (i2c_err != 0) {
        // デバイス未接続: Serial + D4のみ（OLEDは使えない）
        Serial.printf("Wire1 scan: addr=0x%02X err=%d (device not found)\n", OLED_ADDR, i2c_err);
        reportError(false,
            "SSD1306通信不可: Wire1(GPIO26/27)の配線を確認してください",
            nullptr, nullptr, nullptr, nullptr);
        // g_oled_ok は false のまま。以降のエラー表示はSerial+D4のみ
    } else if (!g_oled.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
        // ACKは返ったが初期化シーケンス失敗
        reportError(false,
            "SSD1306初期化失敗: デバイスは応答するが初期化できません",
            nullptr, nullptr, nullptr, nullptr);
    } else {
        g_oled_ok = true;
        g_oled.clearDisplay();
        g_oled.display();
        Serial.println("SSD1306 ready");
    }

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
    buildMenu();  // ユニット種別確定後にメニューを構築

    // ---- [優先度2] DIP設定エラーチェック（DIP1/DIP2 両方OFFは不正・致命的） ----
    if (g_unit_type == 0) {
        reportError(true,
            "DIP設定エラー: DIP1/DIP2 両方OFFは無効。最低1つをONにして再起動",
            "!! DIP ERROR !!",
            "DIP1 & DIP2 both OFF",
            "DIP1=patlite",
            "DIP2=current");
    }

    // ---- E220 モード制御ピン ----
    pinMode(PIN_LORA_M0,  OUTPUT);
    pinMode(PIN_LORA_M1,  OUTPUT);
    // AUXはE220側がオープンドレイン出力: 未接続時に浮遊しないようPULLDOWNを使用
    // E220接続時: idle=HIGH（E220がプル）/ busy=LOW（E220がLOWに引く）
    // E220未接続時: 常にLOW（PULLDOWN）→ e220WaitAuxHighTimeout が確実にタイムアウト
    pinMode(PIN_LORA_AUX, INPUT_PULLDOWN);

    // ---- Serial2 起動（通常動作モード: 9600bps） ----
    serial2Begin(9600);

    // ---- [優先度2.5] E220存在確認（AUX=HIGH タイムアウト3秒） ----
    Serial.println("Waiting for E220 init (AUX=HIGH)...");
    digitalWrite(PIN_LORA_M0, LOW);
    digitalWrite(PIN_LORA_M1, LOW);
    delay(100);
    bool e220_present = e220WaitAuxHighTimeout(3000);
    if (!e220_present) {
        reportError(false,
            "E220未検出: AUX(GPIO6)がHIGHにならない。E220の接続・電源を確認してください",
            "!! E220 ERROR !!",
            "E220 not detected",
            "Check E220 wiring",
            "and power supply");
    } else {
        Serial.println("E220 hardware ready");
        // ---- E220 設定確認・必要なら書き込み ----
        e220ConfigureIfNeeded();
    }

    // ---- パトライトユニット: I2C + TSL2561 初期化 ----
    if (g_unit_type & UNIT_PATLITE) {
        Serial.println("Init I2C (Wire) for TCA9548A + TSL2561...");
        Wire.setSDA(PIN_I2C0_SDA);
        Wire.setSCL(PIN_I2C0_SCL);
        Wire.begin();
        Serial.println("Init TSL2561 sensors...");
        initPatliteSensors();

        // [優先度3] TSL2561初期化失敗チェック（非致命的: D4点灯 + OLED 3秒表示後続行）
        bool se;
        mutex_enter_blocking(&g_mutex);
        se = g_shared.sensor_error;
        mutex_exit(&g_mutex);
        if (se) {
            // 失敗チャンネルをシリアルに列挙
            char detail[22];
            snprintf(detail, sizeof(detail), "fail: CH%s%s%s",
                     g_tsl_ok[0] ? "" : "0 ",
                     g_tsl_ok[1] ? "" : "1 ",
                     g_tsl_ok[2] ? "" : "2");
            reportError(false,
                "TSL2561初期化失敗: TCA9548AおよびTSL2561の配線を確認してください",
                "!! SENSOR ERROR !!",
                "TSL2561 init failed",
                detail,
                "Check TCA9548A/wiring");
        }
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

    // ---- Core1 loop1()開始を許可 ----
    g_setup_done = true;

    Serial.println("=== Ready. Waiting for GW commands. ===");
}

// ===== Main Loop (Core0) =====
void loop() {
    // ---- ボタンポーリング ----
    bool selPr  = pollBtn(PIN_BTN_SEL,  g_selState);
    bool okPr   = pollBtn(PIN_BTN_OK,   g_okState);
    bool backPr = pollBtn(PIN_BTN_BACK, g_backState);

    // ---- UIハンドラ ----
    handleUI(selPr, okPr, backPr);

    // ---- AUX割り込み → コマンド処理（通常モード時のみ） ----
    if (g_uiMode == MODE_NORMAL && g_auxRise) {
        g_auxRise = false;
        if (Serial2.available() >= 3) {
            processCommand();
        }
    }

    // ---- D3 LED（メニュー/編集モード中は点灯） ----
    digitalWrite(PIN_LED_D3, g_uiMode != MODE_NORMAL ? HIGH : LOW);

    // ---- D1 ハートビートパルス（5秒周期、通常モード時のみ・Core0+Core1 両方が生きているときのみ） ----
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
        // メンテ中（黄LED点灯時）またはHWエラー中は青ハートビートを点滅しない
        // g_hw_error中に青が点滅すると「正常動作」に見えてしまうため抑制
        if (core1_alive && g_uiMode == MODE_NORMAL && !g_hw_error) {
            g_d1FlashMs = now;
            digitalWrite(PIN_LED_D1, HIGH);
        }
        // g_hw_error: OLED/E220/DIP等のハードウェアエラーはrebootまで点灯維持
        digitalWrite(PIN_LED_D4, (sensor_err || g_hw_error) ? HIGH : LOW);
    }

    // ---- D1 ハートビートパルス消灯（タイムアウト or メンテモード移行時に即消灯） ----
    if (g_d1FlashMs > 0 && (now - g_d1FlashMs >= D1_FLASH_MS || g_uiMode != MODE_NORMAL)) {
        g_d1FlashMs = 0;
        digitalWrite(PIN_LED_D1, LOW);
    }

    // ---- D2 LoRa受信フラッシュ（100ms後に消灯） ----
    if (g_d2FlashMs > 0 && (millis() - g_d2FlashMs >= D2_FLASH_MS)) {
        g_d2FlashMs = 0;
        digitalWrite(PIN_LED_D2, LOW);
    }

    // ---- OLED更新（200ms間隔） ----
    updateOLED();
}

// ===== Core1 =====
// Wire/センサーはCore0のsetup()で初期化済み。Core1はWireを排他的に使用する。
void setup1() {
    // Core0のsetup()完了を待ってからloop1()を開始する。
    // arduino-picoはCore0のsetup()より前にCore1を起動するため、
    // g_tsl_ok[]等の共有変数をCore0が初期化し終わるまで待つ必要がある。
    //
    // [注意] delay(1) は内部で tud_task() を呼び出し、Core1 から TinyUSB を触ることになる。
    //        Core0 も delay()/Serial.println() で tud_task() を呼ぶため、
    //        両コアが同時に tud_task() を呼ぶと USB CDC 状態が破壊され Serial2.begin() 等が
    //        ハングする原因になる。tight_loop_contents() に変更して競合を排除する。
    //
    // [注意2] multicore_lockout_victim_init() は setup1() のここで呼ぶと Serial2.begin() を
    //         ハングさせる(先頭)か Core1 を起動不能にする(末尾)。
    //         loop1() の初回実行時に呼ぶことで両問題を回避する。
    while (!g_setup_done) {
        tight_loop_contents();  // tud_task を呼ばない
    }
}

void loop1() {
    // flash操作中の協調的一時停止チェック
    // Core0がrequestCore1Pause()を呼んだ場合、RAMに退避してスピン待機する。
    // multicore_lockout_victim_init() は arduino-pico の SIO_IRQ_PROC1 ハンドラと
    // 衝突して Core1 を panic させるため使用しない。
    if (g_core1_pause_req) {
        core1ParkInRam();
        return;
    }

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
        int32_t v_ac = (int32_t)readMCP3208_raw(ADC_CURRENT_CH) - ADC_DC_OFFSET;
        g_current_sum_sq += (int64_t)v_ac * v_ac;
        g_current_count++;

        if (millis() - g_current_window_start >= CURRENT_WINDOW_MS) {
            if (g_current_count > 0) {
                float rms_counts  = sqrtf((float)g_current_sum_sq / (float)g_current_count);
                float current_rms = rms_counts * (3.3f / 4096.0f) * (2000.0f / 33.0f);  // n/RL=2000/33
                // ノイズフロア補正: I_real = sqrt(I_meas^2 - N^2)
                if (CURRENT_NOISE_FLOOR_A > 0.0f) {
                    if (current_rms > CURRENT_NOISE_FLOOR_A) {
                        current_rms = sqrtf(current_rms * current_rms
                                            - CURRENT_NOISE_FLOOR_A * CURRENT_NOISE_FLOOR_A);
                    } else {
                        current_rms = 0.0f;
                    }
                }
                mutex_enter_blocking(&g_mutex);
                g_shared.current_rms = current_rms;
                mutex_exit(&g_mutex);
            }
            g_current_sum_sq       = 0;
            g_current_count        = 0;
            g_current_window_start = millis();
        }

        // Step5: 波形キャプチャ（g_wave_capture フラグ監視）
        if (g_wave_capture) {
            if (!s_prev_capture) s_wave_idx = 0;  // 新キャプチャ開始: インデックスリセット
            if (s_wave_idx < 128) {
                s_wave_local[s_wave_idx++] = (int16_t)v_ac;
                if (s_wave_idx >= 128) {
                    // 128サンプル取得完了 → 共有バッファにコピーして完了通知
                    mutex_enter_blocking(&g_mutex);
                    memcpy(g_shared.wave_buf, s_wave_local, sizeof(s_wave_local));
                    g_shared.wave_ready = true;
                    mutex_exit(&g_mutex);
                    g_wave_capture = false;
                }
            }
            s_prev_capture = true;
        } else {
            s_prev_capture = false;
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
