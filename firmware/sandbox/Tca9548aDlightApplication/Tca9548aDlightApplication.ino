#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_TSL2561_U.h>

// ==== Raspberry Pi Pico I2C pins ====
static const uint8_t PIN_SDA = 20;  // GP20
static const uint8_t PIN_SCL = 21;  // GP21

// ==== TCA9548A I2C address (usually 0x70) ====
#define TCA_ADDR 0x70

// ==== TSL2561 instances (same address OK because TCA separates buses) ====
// 2nd parameter is "sensor ID" used by Adafruit Unified Sensor layer; make them unique
Adafruit_TSL2561_Unified tsl0(TSL2561_ADDR_FLOAT, 1000);
Adafruit_TSL2561_Unified tsl1(TSL2561_ADDR_FLOAT, 1001);
Adafruit_TSL2561_Unified tsl2(TSL2561_ADDR_FLOAT, 1002);

void tcaSelect(uint8_t channel)
{
  if (channel > 7) return;

  Wire.beginTransmission(TCA_ADDR);
  Wire.write(1 << channel);
  Wire.endTransmission();
}

void setupTSL(Adafruit_TSL2561_Unified &tsl, uint8_t ch)
{
  // Select channel before touching the sensor
  tcaSelect(ch);
  delay(2);

  if (!tsl.begin())
  {
    Serial.print("ERROR: TSL2561 not found on CH");
    Serial.println(ch);
    while (1) { delay(1000); }
  }

  // IMPORTANT: configure EACH sensor
  tsl.setGain(TSL2561_GAIN_16X);
  tsl.setIntegrationTime(TSL2561_INTEGRATIONTIME_402MS);

  Serial.print("TSL2561 ready on CH");
  Serial.println(ch);
}

float readLux(Adafruit_TSL2561_Unified &tsl, uint8_t ch)
{
  sensors_event_t event;

  tcaSelect(ch);
  delay(2);

  tsl.getEvent(&event);

  // event.light == 0 could be "too dark" or "saturated", so you may want raw values for debug
  return event.light;
}

void setup()
{
  Serial.begin(115200);
  delay(2000);
  Serial.println("boot");

  Wire.setSDA(PIN_SDA);
  Wire.setSCL(PIN_SCL);
  Wire.begin();

  Serial.println("init sensors...");
  setupTSL(tsl0, 0);
  setupTSL(tsl1, 1);
  setupTSL(tsl2, 2);

  Serial.println("start loop");
}

void loop()
{
  float lux0;
  float lux1;
  float lux2;

  lux0 = readLux(tsl0, 0);
  lux1 = readLux(tsl1, 1);
  lux2 = readLux(tsl2, 2);

  Serial.print("CH0 Lux: ");
  Serial.println(lux0, 2);

  Serial.print("CH1 Lux: ");
  Serial.println(lux1, 2);

  Serial.print("CH2 Lux: ");
  Serial.println(lux2, 2);

  Serial.println("---");
  delay(1000);
}
