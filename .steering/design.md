# Design

## システム構成図
```
[エッジユニット × 最大22台]
  Pico + E220
  ・パトライト監視 (TSL2561 × 3)
  ・電流監視 (CTL-24-CLS + MCP3208)
         ↕ LoRa（ポーリング型）
[ゲートウェイ: app.py（Flask統合アプリ）]
  Raspberry Pi 5 + USB E220
  ┌─ ポーリングスレッド（バックグラウンド）
  │    1分周期: P/Cコマンド → CSV書き込み
  ├─ コマンドキュー（スレッドセーフ）
  │    メンテ操作をWebUI→LoRaスレッドへ橋渡し
  └─ Flaskスレッド（HTTPサーバー）
       ・11台の現在状態一覧
       ・時系列グラフ・品目突合せ
       ・メンテナンス画面（Ping/バージョン/設定/OTA）
         ↕ ファイル（CSV読み取り）
[server_file_copy.py（cron独立実行）]
  ・品目データ取得（SMBファイル同期）
```

## GWアプリケーションアーキテクチャ

```
app.py
├─ polling_thread（daemon=True）
│   ├─ 1分周期: 全機械を順次ポーリング（P/Cコマンド）
│   ├─ データ統合 → CSV書き込み
│   └─ コマンドキューを監視してメンテ操作を実行
│         K(Ping) / H(HW情報) / V(バージョン) / U(OTA)
│         → 結果をresult_queueに返す
└─ Flask routes
    ├─ /           監視画面（11台状態一覧）
    ├─ /graph      時系列グラフ
    ├─ /hinmoku    品目突合せ
    └─ /maintenance メンテナンス画面
        ├─ POST /api/ping      → cmd_queueにKコマンドを積む
        ├─ POST /api/version   → cmd_queueにVコマンドを積む
        ├─ POST /api/hwinfo    → cmd_queueにHコマンドを積む
        └─ POST /api/ota       → cmd_queueにUコマンドを積む（Phase5）
```

- シリアルポート（E220）はpolling_threadが一元管理
- メンテ操作はFlask側からキュー経由で依頼し、結果を受け取る
- 信頼性: systemdの `Restart=always` でプロセス障害時に自動再起動

## GW設定ファイル構造（案）
```yaml
gw_addr: 0x0000           # GW自身のLoRaアドレス（エッジへの送信元アドレス）
machines:
  - name: "A214"
    patlite_addr: 0x0001
    current_addr: 0x000C
    patlite_thresholds:       # パトライト点灯判定閾値（lux）。閾値以上=点灯
      red: 200
      yellow: 200
      green: 200
    current_threshold: 3.0   # 加工中判定閾値（A）。閾値以上=加工中
  - name: "B103"
    patlite_addr: 0x0002
    current_addr: 0x000D
    patlite_thresholds:
      red: 200
      yellow: 200
      green: 200
    current_threshold: 3.0
```
※ 同一ユニットが両機能を兼務する場合は `patlite_addr == current_addr` を設定する

## CSVフォーマット（新設計）
- パス: `data/sensor/<機械名>/YYYY-MM-DD.csv`
- 列: `HH:MM:SS, red_lux, yellow_lux, green_lux, current_A`
  - timestamp列は時刻のみ（HH:MM:SS）。日付はファイル名から取得する
- 記録間隔: 1分

## 状態判定ロジック（GW側・機械ごとに適用）

### Step1: 点灯/消灯判定
```
red点灯   = red_lux   >= patlite_thresholds.red
yellow点灯 = yellow_lux >= patlite_thresholds.yellow
green点灯  = green_lux  >= patlite_thresholds.green
加工中     = current_A  >= current_threshold
```

### Step2: 状態判定（優先順位順）
| 緑 | 電流 | 赤 | 黄 | 判定状態 | 表示色 |
|----|------|----|----|---------|-------|
| ON | any  | any | any | 自動加工中 | green |
| OFF | 加工中 | any | any | 手動加工中 | blue |
| OFF | 加工なし | ON | OFF | アラーム | red |
| OFF | 加工なし | any | ON | 加工完了 | yellow |
| OFF | 加工なし | OFF | OFF | 停止 | gray |

※ 緑点灯が最優先。緑が消えている場合に電流値で自動/手動を区別する。

## 通信プロトコル（確定）

### コマンド一覧（GW → エッジ）
全コマンド共通フォーマット: `[CMD(1byte)]['\r']['\n']`

| コード | 用途 | 備考 |
|--------|------|------|
| `'P'` | パトライトデータ要求 | |
| `'C'` | 電流データ要求 | |
| `'K'` | Ping（死活確認） | |
| `'H'` | ハードウェア・E220設定問い合わせ | |
| `'V'` | ファームウェアバージョン問い合わせ | |
| `'U'` | OTA開始 | Phase5で設計 |

### レスポンス一覧（エッジ → GW）
全レスポンス共通: `[ADDR_H][ADDR_L][CMD][...data...]['\r']['\n']`

```
// Ping応答（5バイト）
[ADDR_H][ADDR_L]['K']['\r']['\n']

// パトライト応答（11バイト）
[ADDR_H][ADDR_L]['P'][RED_H][RED_L][YEL_H][YEL_L][GRN_H][GRN_L]['\r']['\n']
  └ lux値: uint16

// 電流応答（7バイト）
[ADDR_H][ADDR_L]['C'][CUR_H][CUR_L]['\r']['\n']
  └ 電流値: uint16, 0.01A単位（例: 1234 = 12.34A）

// バージョン応答（8バイト）
[ADDR_H][ADDR_L]['V'][MAJOR][MINOR][PATCH]['\r']['\n']

// ハードウェア情報応答（11バイト）
[ADDR_H][ADDR_L]['H'][DIP][E220_ADDR_H][E220_ADDR_L][E220_CH][E220_AIR_RATE][E220_TX_POWER]['\r']['\n']
  └ DIP: bit0=パトライト, bit1=電流, bit2-3=予備

// エラー応答（6バイト）
[ADDR_H][ADDR_L]['E'][ERROR_CODE]['\r']['\n']
  └ 0x01=センサー異常, 0x02=未対応コマンド, 0x03=処理タイムアウト
```

### タイムアウト・再送ポリシー
| 項目 | 値 |
|------|----|
| 応答待ちタイムアウト | 500ms |
| 再送回数 | 1回 |
| 再送後も無応答 | スキップして次のユニットへ |
- 最悪ケース: 22ユニット × (500ms × 2回) = 22秒 → 1分以内に収まる

### ポーリングシーケンス（1分周期・定期自動実行）
```
GW → Edge[patlite_addr]: パトライトデータ要求 ('P')
Edge → GW: red_lux, yellow_lux, green_lux
GW → Edge[current_addr]: 電流データ要求 ('C')
Edge → GW: current_A
→ 2ユニットのデータを統合して1行のCSVに記録
→ 次の機械へ（全機械完了まで繰り返し、1分以内に完了）
```

### メンテナンス操作（Web UIからオンデマンド実行）
K/H/V/U コマンドは定期ポーリングには含まれない。
Webメンテナンス画面から手動トリガーし、コマンドキュー経由でpolling_threadが実行する。

| コマンド | 用途 | 実行タイミング |
|---------|------|-------------|
| `'K'` Ping | 死活確認（全台 or 個別） | 手動 |
| `'H'` HW情報 | DIP状態・E220設定値確認 | 手動 |
| `'V'` バージョン | ファーム確認（全台 or 個別） | 手動 |
| `'U'` OTA | ファームウェア更新 | 手動（Phase5） |

## デュアルコアアーキテクチャ

### Core0（通信・UI担当）
- LoRa通信（E220コマンド受信・レスポンス送信）
- コマンド解釈・モード管理
- ボタン入力・SSD1306表示
- USBシリアル設定

#### AUX割り込みによるコマンド受信
- AUX(GPIO6) のRISINGエッジで割り込み → `volatile bool auxRise = true` をセット
- loop()内で `auxRise && Serial2.available() >= 3` を確認してコマンド処理
  - 受信後のRISING: RXバッファにデータあり → 処理する
  - 送信後のRISING: RXバッファ空 → スルー（ステート変数不要）
- これによりloop()はブロックされず、LEDやボタン処理を並行実行可能
- **起動順序**: `setup()` 内でAUX=HIGHを確認（E220初期化完了）してから `attachInterrupt()` を呼ぶ

### Core1（センサー常時サンプリング担当）
- **パトライト**: TSL2561を3ch高速サイクリング
  - 直近1.5秒間の各センサーmax値を保持
  - パトライトが回転するため瞬時値では消灯誤判定が起きるため
  - 積分時間: 実機確認で決定（13ms/Gain 1X を推奨候補、402msも選択肢）
- **電流**: MCP3208を高速サンプリング（目標~1kHz）
  - 50/60Hz交流のRMS値を常時計算・更新
- **arduino-picoでの実現方法**: `setup1()` / `loop1()` を使用

### コア間データ共有（mutex保護）
```
shared_data {
    float    patlite_max[3];    // 赤・黄・緑の直近1.5秒max値 (lux)
    float    current_rms;       // 最新RMS電流値 (A)
    bool     sensor_error;      // センサー異常フラグ
    uint32_t core1_heartbeat;   // Core1生存確認カウンタ（loop1()内でインクリメント）
}
```
- `#include <pico/mutex.h>` の `mutex_t` を使用
- Core1が書き込み、Core0がコマンド応答時に読み出し

### D1ハートビートLEDによるデュアルコア監視
Core0のみ生存確認では不十分なため、Core1のカウンタを使って両コアを監視する。

```cpp
// Core1: loop1()内でカウンタを更新
void loop1() {
    // センサーサンプリング処理...
    mutex_enter_blocking(&shared_mutex);
    shared_data.core1_heartbeat++;
    mutex_exit(&shared_mutex);
}

// Core0: 1秒ごとにカウンタ変化を確認してD1を制御
uint32_t last_core1_hb = 0;
void updateHeartbeat() {
    uint32_t current_hb = shared_data.core1_heartbeat;  // mutex省略可（読み取りのみ）
    bool core1_alive = (current_hb != last_core1_hb);
    last_core1_hb = current_hb;
    if (core1_alive) digitalWrite(PIN_LED_D1, !digitalRead(PIN_LED_D1));
    // Core1フリーズ時はカウンタが変わらず → D1点滅停止
}
```
- Core0フリーズ → D1点滅停止（従来通り）
- Core1フリーズ → カウンタ不変 → D1点滅停止
- 両方正常 → D1が1秒周期で点滅
- **注意**: Wire / Wire1 / SPI は必ずCore0の `setup()` 内で初期化すること
  （arduino-picoでは `setup1()` は `setup()` 完了後に起動されるため安全）

### ローカルテストモードのセンサー表示
- **通常動作時**: Core1が常時サンプリング → shared_dataにmax/RMSを更新し続ける
- **ローカルテスト時**: スナップショット方式（リアルタイム表示不要）
  - 計測指示 → Core1が短期間サンプリング実行 → 結果を液晶に表示（1回完結）
  - 波形表示: 電流の短期バースト（例: 200ms分）をサンプリングしてSSD1306に描画
  - shared_dataに波形バッファは不要

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
