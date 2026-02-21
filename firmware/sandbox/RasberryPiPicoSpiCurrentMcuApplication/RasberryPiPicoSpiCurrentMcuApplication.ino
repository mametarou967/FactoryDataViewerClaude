#include <SPI.h>

// ==== MCP3208 SPI配線（あなたの結線に合わせて変更） ====
static const int PIN_CS   = 17;
static const int PIN_SCK  = 18;
static const int PIN_MISO = 16;
static const int PIN_MOSI = 19;

// SPIクロック：まずは1MHz（安定したら上げてもOK）
static const uint32_t SPI_HZ = 1000000;

// VREF（MCP3208のVREFに入れている電圧）
static const float VREF_VOLTAGE = 3.3f;

// MCP3208: 12-bit (0..4095)
uint16_t readMCP3208_raw(uint8_t ch)
{
  uint8_t tx0;
  uint8_t tx1;
  uint8_t tx2;

  uint8_t rx0;
  uint8_t rx1;
  uint8_t rx2;

  uint16_t value;

  ch = (uint8_t)(ch & 0x07);

  // MCP3208 コマンド（single-ended）
  // tx0: 0000 0110  (Start=1, SGL=1)
  // tx1: ch<<6      (D2 D1 D0 を上位に)
  // tx2: 0
  tx0 = 0x06;
  tx1 = (uint8_t)(ch << 6);
  tx2 = 0x00;

  SPI.beginTransaction(SPISettings(SPI_HZ, MSBFIRST, SPI_MODE0));
  digitalWrite(PIN_CS, LOW);

  rx0 = SPI.transfer(tx0);
  rx1 = SPI.transfer(tx1);
  rx2 = SPI.transfer(tx2);

  digitalWrite(PIN_CS, HIGH);
  SPI.endTransaction();

  // 12bitの取り出し
  // rx1下位4bitが上位4bit、rx2が下位8bit
  value = (uint16_t)((rx1 & 0x0F) << 8);
  value |= (uint16_t)rx2;

  (void)rx0; // rx0は使わない（ダミー）

  return value;
}

float readMCP3208_voltage(uint8_t ch)
{
  uint16_t raw;
  float voltage;

  raw = readMCP3208_raw(ch);

  // 0..4095 -> 0..VREF
  voltage = ((float)raw * VREF_VOLTAGE) / 4095.0f;

  return voltage;
}

void setup()
{
  Serial.begin(115200);
  delay(2000);
  Serial.println("boot");

  // CS
  pinMode(PIN_CS, OUTPUT);
  digitalWrite(PIN_CS, HIGH);

  // Pico(Arduino-Pico)でSPIピン指定
  SPI.setSCK(PIN_SCK);
  SPI.setRX(PIN_MISO);
  SPI.setTX(PIN_MOSI);
  SPI.begin();

  delay(1000);
  Serial.println("MCP3208 test start");
}

void loop()
{
  float v;
  uint16_t raw;

  // ADS1015のAIN0相当として、MCP3208のCH0を読む
  raw = readMCP3208_raw(0);
  v = ((float)raw * VREF_VOLTAGE) / 4095.0f;

  Serial.print("MCP3208 CH0 raw = ");
  Serial.print(raw);
  Serial.print("  V = ");
  Serial.print(v, 6);
  Serial.println(" V");

  delay(1);
}
