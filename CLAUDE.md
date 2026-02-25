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
| LED ハートビート | GPIO28 | D1: Core0+Core1の両方が生きているときのみ1秒周期点滅 |
| LED LoRa通信状態 | GPIO22 | D2: 正常通信中=点灯、タイムアウト=消灯 |
| LED センサー異常 | GPIO14 | D3: センサーエラー時点灯 |
| LED モード表示 | GPIO15 | D4: 設定/テストモード中点灯 |
| SPI MISO | GPIO16 | MCP3208 |
| SPI CS | GPIO17 | MCP3208 |
| SPI SCK | GPIO18 | MCP3208 |
| SPI MOSI | GPIO19 | MCP3208 |
| I2C0 SDA | GPIO20 | Wire / TCA9548A |
| I2C0 SCL | GPIO21 | Wire / TCA9548A |
| I2C1 SDA | GPIO26 | Wire1 / SSD1306 |
| I2C1 SCL | GPIO27 | Wire1 / SSD1306 |

## 使用ライブラリ
- `Wire.h` / `Adafruit_Sensor.h` / `Adafruit_TSL2561_U.h`（確定）
- `SPI.h`（MCP3208）（確定）
- SSD1306用ライブラリ（`Adafruit SSD1306` または `U8g2`、Step4で確定）

## センサー仕様
- **TCA9548A**: I2Cマルチプレクサ(0x70)、CH0〜2にTSL2561
- **TSL2561**: 積分時間 **13ms / GAIN_1X**（実機確認済み・確定）
  - 13ms: サンプル多（約30回/センサー/1.5秒窓）、工場蛍光灯環境で1000〜2000 lux を正常計測確認
- **MCP3208**: 12bit SPI ADC、VREF=3.3V、1MHz
- **CTL-24-CLS**: ACクランプ + 1.65Vオフセット、RMS計算して電流値(A)を返す

## E220モジュール
使用モジュールは **E220-900T22S/22L(JP)** （日本版）。
海外版と周波数・レジスタマップが異なるため、必ず日本版データシートを参照すること。
- データシート: `docs/DS241012JA_E220-900T22Xv2_Rev1.0.2_JP.pdf`
- 主な違い: チャンネル周波数(920.6+CH×0.2MHz)、REG0エアレートが5ビット(SF/BW複合)、パリティフィールドなし、TX電力が4ビット

## ファイル構成
```
firmware/
  sandbox/        # 既存センサーコード（参照用）
  edge_unit/      # メインスケッチ（実装中）
gateway/
  app.py          # Flask統合アプリ（LoRaポーリング + Web UI + メンテ画面）（再実装予定）
  server_file_copy.py  # SMBファイル同期（cron独立実行）
  lora_logger.py  # プロトタイプ版（参照用・廃止予定）
hardware/         # KiCadスケマティック
docs/             # データシート等（E220 JP版など）
tools/            # 開発・設定・テスト用スクリプト（gw_e220_config.py, test_edge.py）
.steering/        # 仕様・設計・タスク管理
```
