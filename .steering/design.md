# Design

## システム構成図
```
[エッジユニット × 最大22台]
  Pico + E220
  ・パトライト監視 (TSL2561 × 3)
  ・電流監視 (CTL-24-CLS + MCP3208)
         ↕ LoRa（ポーリング型）
[ゲートウェイ]
  Raspberry Pi 5 + USB E220
  ・ポーリング制御
  ・データ統合・CSV保存
  ・品目データ取得（SMB）
         ↕
[Webアプリ (Flask)]
  ・11台の現在状態一覧
  ・時系列グラフ
  ・品目突合せ
```

## GW設定ファイル構造（案）
```yaml
machines:
  - name: "A214"
    patlite_addr: 0x0001
    current_addr: 0x000C
    current_thresholds:       # 状態判定の閾値（機械ごとに異なる）
      idle: 1.0
      auto: 5.0
      manual: 10.0
  - name: "B103"
    patlite_addr: 0x0002
    current_addr: 0x000D
    ...
```

## CSVフォーマット（新設計）
- パス: `data/sensor/<機械名>/YYYY-MM-DD.csv`
- 列: `timestamp, red_lux, yellow_lux, green_lux, current_A`
- 記録間隔: 1分

## 通信プロトコル（設計中）
→ パケットフォーマットは別途確定予定

### ポーリングシーケンス
```
GW → Edge[patlite_addr]: パトライトデータ要求
Edge → GW: red_lux, yellow_lux, green_lux
GW → Edge[current_addr]: 電流データ要求
Edge → GW: current_A
→ 2ユニットのデータを統合して1行のCSVに記録
→ 次の機械へ（22ユニット完了まで繰り返し、1分以内に完了）
```

## エッジ動作モード
| モード | 遷移条件 |
|--------|----------|
| 通常動作 | 起動時のデフォルト |
| 設定モード | ボタン操作 or LoRaコマンド or USBコマンド |
| ローカルテスト | 設定メニューから選択 |
| OTA受信 | GWからのOTAコマンド（将来実装） |

## 既存GWとの差分（プロトタイプ → 本番）
| 項目 | プロトタイプ | 本番 |
|------|------------|------|
| 台数 | 2台 | 最大22台 |
| 通信方式 | イベント駆動 | ポーリング型 |
| データ保存 | 1ファイル | 機械ごとサブディレクトリ |
| 状態判定閾値 | app.py内ハードコード | 設定ファイルで管理 |
