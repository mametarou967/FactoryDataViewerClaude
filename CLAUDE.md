# プロジェクト概要
工場11台の機械を監視するシステム。エッジユニット（Pico+LoRa）→ ゲートウェイ（RPi5）→ Web表示。
詳細仕様は `.steering/` を参照。

## 環境
- ターゲットボード: Raspberry Pi Pico (RP2040)
- 開発環境: Arduino IDE / Earle Philhowerのarduino-pico

## Pin Assignment
| 用途 | GPIO | 備考 |
|------|------|------|
| LoRa M0 | GPIO2 | E220 |
| LoRa M1 | GPIO3 | E220 |
| LoRa TX | GPIO4 | Serial2 |
| LoRa RX | GPIO5 | Serial2 |
| LoRa AUX | GPIO6 | E220 |
| ボタン 選択(灰) | GPIO7 | SW2 |
| ボタン 決定(赤) | GPIO8 | SW3 |
| ボタン 戻る(黒) | GPIO9 | SW4 |
| DIP1 パトライト判定 | GPIO10 | SW5 |
| DIP2 電流判定 | GPIO11 | SW5 |
| DIP3 予備 | GPIO12 | SW5 |
| DIP4 予備 | GPIO13 | SW5 |
| LED1-4 | GPIO14/15/22/28 | D3/D4/D2/D1 |
| SPI MISO | GPIO16 | MCP3208 |
| SPI CS | GPIO17 | MCP3208 |
| SPI SCK | GPIO18 | MCP3208 |
| SPI MOSI | GPIO19 | MCP3208 |
| I2C0 SDA | GPIO20 | Wire / TCA9548A |
| I2C0 SCL | GPIO21 | Wire / TCA9548A |
| I2C1 SDA | GPIO26 | Wire1 / SSD1306 |
| I2C1 SCL | GPIO27 | Wire1 / SSD1306 |

## 使用ライブラリ（確定済み）
- `Wire.h` / `Adafruit_Sensor.h` / `Adafruit_TSL2561_U.h`
- `SPI.h`（MCP3208）
- SSD1306用ライブラリ（未確定）

## センサー仕様
- **TCA9548A**: I2Cマルチプレクサ(0x70)、CH0〜2にTSL2561
- **TSL2561**: Gain 16X、積分時間 402ms
- **MCP3208**: 12bit SPI ADC、VREF=3.3V、1MHz
- **CTL-24-CLS**: ACクランプ + 1.65Vオフセット、RMS計算して電流値(A)を返す

## ファイル構成
```
firmware/
  sandbox/        # 既存センサーコード（参照用）
  edge_unit/      # メインスケッチ（実装予定）
gateway/
  lora_logger.py  # LoRa受信・CSV保存（再実装予定）
  server_file_copy.py  # SMBファイル同期
  app.py          # Webアプリ（再実装予定）
hardware/         # KiCadスケマティック
.steering/        # 仕様・設計・タスク管理
```
