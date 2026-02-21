from flask import Flask, render_template, abort, send_file
import csv
import os
from datetime import datetime, timedelta, time

import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt
import matplotlib.font_manager as fm
from collections import defaultdict
from calendar import monthrange


app = Flask(__name__)
DATA_DIR = "data/sensor"
HINMOKU_SUBDIR = "../hinmoku"  # 品目CSVのサブディレクトリ名（data/hinmoku/）

# 点灯/消灯の閾値
THRESHOLDS = {
    "red": 200,
    "yellow": 200,
    "green": 200
}
CURRENT_THRESHOLD = 3.0

# 点灯・状態判定
def get_light_status(red, yellow, green, current):
    def is_on(color, value):
        return value >= THRESHOLDS[color]

    status = {
        "red": "点灯" if is_on("red", red) else "消灯",
        "yellow": "点灯" if is_on("yellow", yellow) else "消灯",
        "green": "点灯" if is_on("green", green) else "消灯"
    }

    machine_action = "加工中" if current >= CURRENT_THRESHOLD else "加工なし"

    # 状態判定
    state, color = "停止", "gray"  # 色なし
    r, y, g, m = status["red"], status["yellow"], status["green"], machine_action
    
    if r == "消灯" and y == "消灯" and g == "消灯" and m == "加工なし":
        state, color = "停止", "gray"
    elif r == "消灯" and y == "消灯" and g == "点灯" and m == "加工なし":
        state, color = "自動加工中", "green"
    elif r == "消灯" and y == "点灯" and g == "消灯" and m == "加工なし":
        state, color = "加工完了", "yellow"
    elif r == "消灯" and y == "点灯" and g == "点灯" and m == "加工なし":
        state, color = "自動加工中", "green"
    elif r == "点灯" and y == "消灯" and g == "消灯" and m == "加工なし":
        state, color = "アラーム", "red"
    elif r == "点灯" and y == "消灯" and g == "点灯" and m == "加工なし":
        state, color = "自動加工中", "green"
    elif r == "点灯" and y == "点灯" and g == "消灯" and m == "加工なし":
        state, color = "加工完了", "yellow"
    elif r == "点灯" and y == "点灯" and g == "点灯" and m == "加工なし":
        state, color = "自動加工中", "green"
    elif r == "消灯" and y == "消灯" and g == "消灯" and m == "加工中":
        state, color = "手動加工中", "blue"
    elif r == "消灯" and y == "消灯" and g == "点灯" and m == "加工中":
        state, color = "自動加工中", "green"
    elif r == "消灯" and y == "点灯" and g == "消灯" and m == "加工中":
        state, color = "手動加工中", "blue"
    elif r == "消灯" and y == "点灯" and g == "点灯" and m == "加工中":
        state, color = "自動加工中", "green"
    elif r == "点灯" and y == "消灯" and g == "消灯" and m == "加工中":
        state, color = "手動加工中", "blue"
    elif r == "点灯" and y == "消灯" and g == "点灯" and m == "加工中":
        state, color = "自動加工中", "green"
    elif r == "点灯" and y == "点灯" and g == "消灯" and m == "加工中":
        state, color = "手動加工中", "blue"
    elif r == "点灯" and y == "点灯" and g == "点灯" and m == "加工中":
        state, color = "自動加工中", "green"

    return status, machine_action, state, color

# 最新データ取得
def get_latest_data():
    now = datetime.now()
    threshold = now - timedelta(minutes=5)
    latest_row = None

    for filename in sorted(os.listdir(DATA_DIR), reverse=True):
        if not filename.endswith(".csv"):
            continue
        filepath = os.path.join(DATA_DIR, filename)
        with open(filepath, newline='', encoding='utf-8') as f:
            reader = csv.reader(f)
            for row in reversed(list(reader)):
                try:
                    row_time = datetime.strptime(f"{filename[:-4]} {row[0]}", "%Y-%m-%d %H:%M:%S")
                except:
                    continue
                if threshold <= row_time <= now:
                    return {
                        "time": row[0],
                        "red": float(row[1]),
                        "yellow": float(row[2]),
                        "green": float(row[3]),
                        "current": float(row[4]),
                        "timestamp": row_time.strftime("%Y-%m-%d %H:%M:%S")
                    }
    return None

def read_hinmoku_csv(date_str):
    """
    品目CSV: data/hinmoku/A214_YYYYMMDD_.csv を読み、(headers, rows) を返す。
    rows は 1行=リスト。1行目は見出し。
    文字コードは cp932 優先、失敗時に utf-8 にフォールバック。
    """
    yyyymmdd = datetime.strptime(date_str, "%Y-%m-%d").strftime("%Y%m%d")
    expected_name = f"A214_{yyyymmdd}.csv"
    dirpath = os.path.join(DATA_DIR, HINMOKU_SUBDIR)
    filepath = os.path.join(dirpath, expected_name)

    if not os.path.isdir(dirpath) or not os.path.exists(filepath):
        return None, None, expected_name  # 見つからない

    rows = []
    # cp932 -> utf-8 の順でトライ
    tried_encodings = ["cp932", "utf-8"]
    last_err = None
    for enc in tried_encodings:
        try:
            with open(filepath, newline="", encoding=enc) as f:
                reader = csv.reader(f)
                for row in reader:
                    if row:
                        rows.append(row)
            break
        except Exception as e:
            rows = []
            last_err = e
            continue

    if not rows:
        # 空 or 読めない
        return None, None, expected_name

    headers = rows[0]
    records = rows[1:]
    return headers, records, expected_name

def set_japanese_font():
    """日本語フォント設定（存在すれば適用）"""
    font_path = "/usr/share/fonts/truetype/vlgothic/VL-Gothic-Regular.ttf"
    if os.path.exists(font_path):
        plt.rcParams['font.family'] = fm.FontProperties(fname=font_path).get_name()


def _day_range(date_str):
    base_date = datetime.strptime(date_str, "%Y-%m-%d")
    start = datetime.combine(base_date, datetime.strptime("00:00:00", "%H:%M:%S").time())
    end   = start + timedelta(days=1)
    return start, end


def _load_minute_colors(date_str, start_dt=None, end_dt=None, include_gray=True):
    """
    data/<date_str>.csv を読み、分単位の色辞書 {datetime: color} を返す。
    start_dt/end_dt が指定されれば [start_dt, end_dt) にクリップして格納。
    include_gray=False の場合は 'gray' を除外。
    """
    csv_path = os.path.join(DATA_DIR, f"{date_str}.csv")
    if not os.path.exists(csv_path):
        return None  # データなし

    day_start, day_end = _day_range(date_str)

    # クリップ（無指定なら一日分）
    s = max(start_dt, day_start) if start_dt else day_start
    e = min(end_dt,   day_end)   if end_dt   else day_end

    minute_color = {}
    with open(csv_path, newline='', encoding='utf-8') as f:
        for row in csv.reader(f):
            if len(row) < 5:
                continue
            try:
                t = datetime.strptime(date_str + " " + row[0], "%Y-%m-%d %H:%M:%S")
            except Exception:
                continue

            # クリップ範囲外は無視
            if not (s <= t < e):
                continue

            try:
                r, y, g, c = float(row[1]), float(row[2]), float(row[3]), float(row[4])
                _, _, _, color = get_light_status(r, y, g, c)
                if (color == "gray") and (not include_gray):
                    continue
                minute_color[t] = color
            except Exception:
                continue

    return minute_color


def _render_day_timeline(date_str, minute_color, out_png_path, title):
    """
    1日横棒を描画。minute_color に入っている分だけ色を塗る。
    """
    set_japanese_font()
    day_start, day_end = _day_range(date_str)

    plt.figure(figsize=(14, 2))
    current = day_start
    while current < day_end:
        color = minute_color.get(current)
        if color:
            left = (current - day_start).total_seconds()
            plt.barh(0, 60, left=left, height=0.5, color=color)
        current += timedelta(minutes=1)

    # 目盛り（毎時）
    xticks, xticklabels = [], []
    hour = day_start
    while hour <= day_end:
        xticks.append((hour - day_start).total_seconds())
        xticklabels.append("24:00" if hour == day_end else hour.strftime("%H:%M"))
        hour += timedelta(hours=1)

    plt.xticks(xticks, xticklabels)
    plt.yticks([])
    plt.xlim(0, (day_end - day_start).total_seconds())
    plt.title(title)
    plt.tight_layout()

    os.makedirs("static", exist_ok=True)
    if os.path.exists(out_png_path):
        os.remove(out_png_path)
    plt.savefig(out_png_path)
    plt.close()


def generate_graph_image_unified(
    date_str,
    start_dt=None,
    end_dt=None,
    out_png_path=None,
    include_gray=True,
    skip_if_up_to_date=True,
):
    """
    1本化された描画関数。
    - 日全体: start_dt/end_dt を渡さない
    - 区間のみ: start_dt/end_dt を渡す（[start_dt, end_dt)）
    - out_png_path 未指定時はデイリーの既定パスを使う
    - デイリーはCSVが新しければ再描画、最新ならスキップ（skip_if_up_to_date=True）
    """
    csv_path = os.path.join(DATA_DIR, f"{date_str}.csv")
    if not os.path.exists(csv_path):
        return False

    # 既定の出力先
    if out_png_path is None:
        image_filename = f"{date_str}_graph.png"
        out_png_path = os.path.join("static", image_filename)

    # 「日全体」かつ「最新ならスキップ」だけ最適化
    is_full_day = (start_dt is None and end_dt is None)
    if skip_if_up_to_date and is_full_day and os.path.exists(out_png_path):
        if os.path.getmtime(csv_path) <= os.path.getmtime(out_png_path):
            return True  # 画像が最新

    # 分ごとの色割り当てを取得
    minute_color = _load_minute_colors(
        date_str,
        start_dt=start_dt,
        end_dt=end_dt,
        include_gray=include_gray
    )
    if not minute_color:
        return False

    # タイトル
    if is_full_day:
        title = f"{date_str} 状態推移グラフ"
    else:
        # 表示は当日内の時刻だけで十分
        s = start_dt.strftime("%H:%M")
        e = end_dt.strftime("%H:%M")
        title = f"{date_str} 品目時間帯グラフ（{s}〜{e}）"

    _render_day_timeline(date_str, minute_color, out_png_path, title)
    return True

def generate_graph_image(date):
    # 互換ラッパー：そのまま呼ばれても動くように
    return generate_graph_image_unified(date_str=date)

def generate_graph_image_for_interval(date_str, start_dt, end_dt, out_png_path):
    # 互換ラッパー：include_gray のデフォルトはこれまでと同じ挙動（灰色も描画）
    return generate_graph_image_unified(
        date_str=date_str,
        start_dt=start_dt,
        end_dt=end_dt,
        out_png_path=out_png_path,
        include_gray=True,
        # 区間は同じファイル名で上書きする可能性があるので常に再描画を推奨
        skip_if_up_to_date=False,
    )

def generate_graph_image_for_intervals(date_str, intervals, out_png_path):
    """
    複数区間の合成グラフを1枚に描画。
    """
    if not intervals:
        return False

    # 分ごとの色を区間ごとに読み、ORマージ
    merged = {}
    for s_dt, e_dt in intervals:
        mc = _load_minute_colors(date_str, start_dt=s_dt, end_dt=e_dt, include_gray=True)
        if not mc:
            continue
        merged.update(mc)

    if not merged:
        return False

    # タイトル：最初と最後の時刻を表示
    s_label = intervals[0][0].strftime("%H:%M")
    e_label = intervals[-1][1].strftime("%H:%M")
    title = f"{date_str} 品目時間帯グラフ（{s_label}〜{e_label}／{len(intervals)}区間）"

    _render_day_timeline(date_str, merged, out_png_path, title)
    return True


# --- 追記: 柔軟な日時パーサ（秒あり/なしを許容） ---
def parse_flexible_dt(s):
    """
    "YYYY/M/D H:M[:S]" 形式（ゼロ詰め無しOK、秒あり/なし両対応）を受け付けて datetime を返す。
    例: "2025/8/4 8:46", "2025/08/04 08:46:13"
    """
    s = s.strip()
    fmts = [
        "%Y/%m/%d %H:%M:%S",
        "%Y/%m/%d %H:%M",
        "%Y/%-m/%-d %-H:%-M:%-S",  # Linux系のゼロ無し。Windowsでは無視されるが例として残す
        "%Y/%-m/%-d %-H:%-M",
    ]
    for fmt in fmts:
        try:
            return datetime.strptime(s, fmt)
        except Exception:
            continue
    # 上の %- 指定は環境依存。最後に標準的な置換で再トライ
    try:
        # ゼロ詰めして秒なしをまず試す
        parts = s.replace("/", " ").replace(":", " ").split()
        # ["YYYY","M","D","H","M"(,"S")]
        if len(parts) >= 5:
            Y, M, D, h, m = parts[:5]
            sec = parts[5] if len(parts) >= 6 else "00"
            canon = f"{int(Y):04d}/{int(M):02d}/{int(D):02d} {int(h):02d}:{int(m):02d}:{int(sec):02d}"
            return datetime.strptime(canon, "%Y/%m/%d %H:%M:%S")
    except Exception:
        pass
    raise ValueError("日時の形式が不正です")

def extract_intervals_from_row(date_str, row):
    """
    新CSV1行から最大5つの区間[(start_dt, end_dt), ...]を返す。
    列: 9=開始1,10=停止1, 11=開始2,12=停止2, ..., 17=開始5,18=停止5
    """
    base_date = datetime.strptime(date_str, "%Y-%m-%d").date()
    day_start = datetime.combine(base_date, time(0, 0, 0))
    day_end   = datetime.combine(base_date, time(23, 59, 59))
    now = datetime.now()

    intervals = []
    # 5ペアを走査
    for i in range(5):
        s_idx = 9 + 2*i
        e_idx = 10 + 2*i
        if len(row) <= s_idx:
            break
        s_raw = (row[s_idx] or "").strip()
        e_raw = (row[e_idx] if len(row) > e_idx else "")
        e_raw = (e_raw or "").strip()

        if not s_raw:
            continue  # 開始が空→このペアなし

        try:
            s_dt = parse_flexible_dt(s_raw)
        except Exception:
            continue

        # 停止の補完
        e_dt = None
        if e_raw:
            try:
                e_dt = parse_flexible_dt(e_raw)
            except Exception:
                e_dt = None
        if e_dt is None:
            e_dt = min(now, day_end) if base_date == now.date() else day_end

        # 当日範囲にクリップ
        s_dt = max(s_dt, day_start)
        e_dt = min(e_dt, day_end)

        if e_dt <= s_dt:
            e_dt = min(s_dt + timedelta(minutes=1), day_end)

        # 最終的に当日と重なるものだけ採用
        if s_dt < e_dt:
            intervals.append((s_dt, e_dt))

    # 時刻順に整列
    intervals.sort(key=lambda t: t[0])
    return intervals


def resolve_item_interval(date_str, start_raw, end_raw):
    """
    品目CSVの着手・完了文字列から区間 [start_dt, end_dt) を決める。
    - start_raw が不正なら例外を投げる（ここは必須）
    - end_raw が空 or 不正なら“当日末(23:59:59)”まで。対象日が今日なら“現在時刻”まで。
    """
    base_date = datetime.strptime(date_str, "%Y-%m-%d").date()
    day_start = datetime.combine(base_date, time(0, 0, 0))
    day_end   = datetime.combine(base_date, time(23, 59, 59))

    # start は必須（不正なら例外）
    start_dt = parse_flexible_dt((start_raw or "").strip())

    # end は任意（未記入 or 不正なら補完）
    end_dt = None
    end_raw = (end_raw or "").strip()
    if end_raw:
        try:
            end_dt = parse_flexible_dt(end_raw)
        except Exception:
            end_dt = None

    if end_dt is None:
        # 未記入 or 不正 → 今日なら now まで、過去日なら当日末まで
        now = datetime.now()
        if base_date == now.date():
            end_dt = min(now, day_end)
        else:
            end_dt = day_end

    # 最低限、start < end になるように当日末でクリップ
    if end_dt <= start_dt:
        # まれに同時刻などの異常値は、1分だけ足して日内上限でクリップ
        end_dt = min(start_dt + timedelta(minutes=1), day_end)

    return start_dt, end_dt

def get_current_processing_items(now=None):
    if now is None:
        now = datetime.now()
    today_str = now.strftime("%Y-%m-%d")

    headers, records, expected_name = read_hinmoku_csv(today_str)
    result = {"has_csv": False, "expected": expected_name, "items": []}
    if not headers or records is None:
        return result
    result["has_csv"] = True

    for idx, row in enumerate(records, start=1):
        if len(row) < 19:  # 最低でも停止5までの列がある想定
            # 多少短いCSVでも、開始1(9)があれば動くように下限は緩くしてもOK
            pass
        intervals = extract_intervals_from_row(today_str, row)
        if not intervals:
            continue

        # now がどれかの区間に含まれる？
        in_any = any(s <= now < e for s, e in intervals)
        if not in_any:
            continue

        # 表示用：区間を "HH:MM-HH:MM / ..." につなぐ
        intervals_str = " / ".join(f"{s.strftime('%H:%M')}-{e.strftime('%H:%M')}" for s, e in intervals)

        info = {
            "kikai_no": row[0],
            "seiban": row[1],
            "tehai_no": row[2],
            "hinmoku_no": row[3],
            "hinmoku_name": row[4],
            "intervals": intervals_str,
            "status": (row[8] if len(row) > 8 else "")
        }
        result["items"].append({"index": idx, "info": info})

    return result



# --- 追記: 区間限定の状態別集計ユーティリティ ---
def summarize_states_for_interval(date_str, start_dt, end_dt):
    """
    data/<date_str>.csv の 1分単位データを読み、[start_dt, end_dt) の区間だけ
    状態別合計秒数を算出して返す（dict: state -> 秒）。
    区間が当日の 0:00〜24:00 をはみ出していれば当日内にクリップする。
    """
    csv_path = os.path.join(DATA_DIR, f"{date_str}.csv")
    if not os.path.exists(csv_path):
        return None  # データなし

    base_date = datetime.strptime(date_str, "%Y-%m-%d")
    day_start = datetime.combine(base_date, datetime.strptime("00:00:00", "%H:%M:%S").time())
    day_end   = day_start + timedelta(days=1)

    # クリップ
    s = max(start_dt, day_start)
    e = min(end_dt,   day_end)
    if not (s < e):
        # 重なりなし
        return {k: 0 for k in ["自動加工中", "手動加工中", "加工完了", "アラーム", "停止"]}

    states = ["自動加工中", "手動加工中", "加工完了", "アラーム", "停止"]
    secs = {state: 0 for state in states}

    with open(csv_path, newline="", encoding="utf-8") as f:
        for row in csv.reader(f):
            if len(row) < 5:
                continue
            try:
                t = datetime.strptime(date_str + " " + row[0], "%Y-%m-%d %H:%M:%S")
            except Exception:
                # 時刻列が壊れている行はスキップ
                continue

            if not (s <= t < e):
                continue

            try:
                r, y, g, c = float(row[1]), float(row[2]), float(row[3]), float(row[4])
                _, _, state, _ = get_light_status(r, y, g, c)
                secs[state] += 60  # 1分分加算
            except Exception:
                continue

    return secs

def summarize_states_for_intervals(date_str, intervals):
    """
    複数区間の合計（状態別・秒）を返す。
    """
    if not intervals:
        return {k: 0 for k in ["自動加工中", "手動加工中", "加工完了", "アラーム", "停止"]}
    total = {k: 0 for k in ["自動加工中", "手動加工中", "加工完了", "アラーム", "停止"]}
    for s_dt, e_dt in intervals:
        secs = summarize_states_for_interval(date_str, s_dt, e_dt)
        if secs is None:
            continue
        for k, v in secs.items():
            total[k] += v
    return total

def generate_graph_image_for_intervals(date_str, intervals, out_png_path):
    """
    複数区間の合成グラフを1枚に描画。
    """
    if not intervals:
        return False

    # 分ごとの色を区間ごとに読み、ORマージ
    merged = {}
    for s_dt, e_dt in intervals:
        mc = _load_minute_colors(date_str, start_dt=s_dt, end_dt=e_dt, include_gray=True)
        if not mc:
            continue
        merged.update(mc)

    if not merged:
        return False

    # タイトル：最初と最後の時刻を表示
    s_label = intervals[0][0].strftime("%H:%M")
    e_label = intervals[-1][1].strftime("%H:%M")
    title = f"{date_str} 品目時間帯グラフ（{s_label}〜{e_label}／{len(intervals)}区間）"

    _render_day_timeline(date_str, merged, out_png_path, title)
    return True

def summarize_states_full_day_hours(date_str):
    """
    data/<date_str>.csv を読み、日全体（24h）の状態別合計時間（h）を小数2桁で返す。
    データが無ければ None。
    """
    csv_path = os.path.join(DATA_DIR, f"{date_str}.csv")
    if not os.path.exists(csv_path):
        return None
    states = ["自動加工中", "手動加工中", "加工完了", "アラーム", "停止"]
    secs = {s: 0 for s in states}
    with open(csv_path, newline='', encoding='utf-8') as f:
        for row in csv.reader(f):
            if len(row) < 5:
                continue
            try:
                r, y, g, c = float(row[1]), float(row[2]), float(row[3]), float(row[4])
                _, _, state, _ = get_light_status(r, y, g, c)
                secs[state] += 60
            except Exception:
                continue
    return {k: round(v/3600.0, 2) for k, v in secs.items()}

@app.route("/")
def index():
    # --- ライト/電流の最新値（過去5分） ---
    latest = get_latest_data()

    status_debug = None
    status_summary = None

    if latest:
        lights, machine_action, state, color = get_light_status(
            latest["red"], latest["yellow"], latest["green"], latest["current"]
        )

        # 旧表示（デバッグ用：点灯/消灯の3円）
        status_debug = {
            **lights,
            "current": latest["current"],
            "timestamp": latest["timestamp"]
        }

        # 新表示（上部：状態1円）
        status_summary = {
            "state": state,               # 自動加工中/手動加工中/加工完了/アラーム/停止
            "color": color,               # green/yellow/red/blue/gray
            "current": latest["current"], # 必要なら下に表示できるよう残す
            "timestamp": latest["timestamp"]
        }

    # --- 右カラム：現在の加工状況（今日のhinmoku）---
    now = datetime.now()
    current_work = get_current_processing_items(now=now)
    today_str = now.strftime("%Y-%m-%d")

    # --- 年度カレンダー（4月～翌年3月） ---
    if now.month >= 4:
        fiscal_start = datetime(now.year, 4, 1)
    else:
        fiscal_start = datetime(now.year - 1, 4, 1)
    fiscal_end = fiscal_start.replace(year=fiscal_start.year + 1) - timedelta(days=1)

    # 存在するCSV日付・月の判定
    existing_days = set()
    existing_months = set()
    for fname in os.listdir(DATA_DIR):
        if fname.endswith(".csv"):
            try:
                date_obj = datetime.strptime(fname[:-4], "%Y-%m-%d")
                if fiscal_start <= date_obj <= fiscal_end:
                    existing_days.add(date_obj.date())
                    existing_months.add(date_obj.strftime("%Y-%m"))
            except:
                continue

    # カレンダーデータ構築
    calendar = {}
    current = fiscal_start
    while current <= fiscal_end:
        ym = current.strftime("%Y-%m")
        if ym not in calendar:
            calendar[ym] = {
                "month_link": ym in existing_months,
                "weeks": [[]]
            }

        week = calendar[ym]["weeks"][-1]
        if len(week) == 0 and current.weekday() != 0:
            week.extend([None] * current.weekday())

        week.append({
            "day": current.day,
            "date": current.strftime("%Y-%m-%d"),
            "link": current.date() in existing_days
        })

        if current.weekday() == 6:
            calendar[ym]["weeks"].append([])

        current += timedelta(days=1)

    return render_template(
        "index.html",
        status_summary=status_summary,   # ★追加（上部1円）
        status=status_debug,             # ★従来（下部へ移動するデバッグ用）
        thresholds=THRESHOLDS,
        current_threshold=CURRENT_THRESHOLD,
        calendar=calendar,
        current_work=current_work,
        today=today_str
    )


@app.route("/month/<year_month>/overview")
def show_month_overview(year_month):
    """月俯瞰：2列（左=日別サマリ、右=日別グラフ）、DD_MAX行"""
    try:
        target_month = datetime.strptime(year_month, "%Y-%m")
    except ValueError:
        abort(404)

    year = target_month.year
    month = target_month.month
    dd_max = monthrange(year, month)[1]

    items = []
    for day in range(1, dd_max + 1):
        date_str = f"{year_month}-{day:02d}"
        csv_path = os.path.join(DATA_DIR, f"{date_str}.csv")

        durations = None
        image_filename = None

        if os.path.exists(csv_path):
            # 左列：日別サマリ（時間）
            durations = summarize_states_full_day_hours(date_str)  # dict or None（通常はdict）
            # 右列：日別グラフ
            generate_graph_image(date_str)  # 既存ならスキップ
            image_filename = f"{date_str}_graph.png"

        items.append({
            "date": date_str,
            "durations": durations,          # None のときは「データなし」を表示
            "image_filename": image_filename # None のときは「グラフなし」を表示
        })

    return render_template("month/overview.html", year_month=year_month, items=items)

@app.route("/month/<year_month>/graph")
def show_month_graph(year_month):
    try:
        month_date = datetime.strptime(year_month, "%Y-%m")
    except:
        abort(404)

    year = month_date.year
    month = month_date.month
    day = 1
    images = []

    while True:
        try:
            current_date = datetime(year, month, day)
        except:
            break
        date_str = current_date.strftime("%Y-%m-%d")
        csv_path = os.path.join(DATA_DIR, f"{date_str}.csv")
        image_filename = f"{date_str}_graph.png"
        image_path = os.path.join("static", image_filename)

        # 存在するCSVファイルについてのみ描画
        if os.path.exists(csv_path):
            generate_graph_image(date_str)
            images.append({
                "date": date_str,
                "image_filename": image_filename
            })

        day += 1

    if not images:
        abort(404)

    return render_template("month/graph.html", year_month=year_month, images=images)


@app.route("/month/<year_month>/summary")
def show_month_summary(year_month):
    try:
        target_month = datetime.strptime(year_month, "%Y-%m")
    except ValueError:
        abort(404)

    states = ["自動加工中", "手動加工中", "加工完了", "アラーム", "停止"]

    # -----------------------------
    # 9H(8:00-17:00) ベース集計
    # -----------------------------
    summaries_9h = {state: [] for state in states}
    labels_9h = []  # 24h表と同じく、データがある日だけ並べる

    # 24h（既存）集計
    summaries_24h = {state: [] for state in states}
    labels_24h = []

    # 該当月の .csv だけを処理（ファイル名=YYYY-MM-DD.csv）
    for fname in sorted(os.listdir(DATA_DIR)):
        if not fname.endswith(".csv"):
            continue
        try:
            date_obj = datetime.strptime(fname[:-4], "%Y-%m-%d")
        except ValueError:
            continue
        if date_obj.strftime("%Y-%m") != year_month:
            continue

        date_str = date_obj.strftime("%Y-%m-%d")
        filepath = os.path.join(DATA_DIR, fname)

        # --- 9H（8:00-17:00） ---
        start_dt = datetime.combine(date_obj.date(), time(8, 0, 0))
        end_dt = datetime.combine(date_obj.date(), time(17, 0, 0))
        secs_9h = summarize_states_for_interval(date_str, start_dt, end_dt)
        if secs_9h is not None:
            labels_9h.append(date_str)
            for state in states:
                summaries_9h[state].append(round(secs_9h.get(state, 0) / 3600.0, 2))

        # --- 24H（既存：日全体）---
        labels_24h.append(date_str)
        durations_sec = {state: 0 for state in states}
        with open(filepath, newline='', encoding='utf-8') as f:
            for row in csv.reader(f):
                if len(row) < 5:
                    continue
                try:
                    r, y, g, c = float(row[1]), float(row[2]), float(row[3]), float(row[4])
                    _, _, state, _ = get_light_status(r, y, g, c)
                    durations_sec[state] += 60
                except:
                    continue
        for state in states:
            summaries_24h[state].append(round(durations_sec[state] / 3600.0, 2))

    if not labels_24h and not labels_9h:
        abort(404, description="指定された月にデータが見つかりませんでした")

    # -----------------------------
    # 9H表の合計（行合計/列合計/総合計）
    # -----------------------------
    row_totals_9h = {state: round(sum(summaries_9h[state]), 2) for state in states}

    num_days_9h = len(labels_9h)
    column_totals_9h = []
    
    working_states = ["自動加工中", "手動加工中", "加工完了"]
    
    for i in range(num_days_9h):
        day_sum = 0.0
        for state in states:
            if i < len(summaries_9h[state]):
                day_sum += summaries_9h[state][i]
        column_totals_9h.append(round(day_sum, 2))
    grand_total_9h = round(sum(row_totals_9h.values()), 2)
    
    # --- 9H 稼働時間合計(時間)（自動+手動+完了）---
    working_states = ["自動加工中", "手動加工中", "加工完了"]

    num_days_9h = len(labels_9h)

    working_column_totals_9h = []
    for i in range(num_days_9h):
        work_sum = 0.0
        for s in working_states:
            if i < len(summaries_9h[s]):
                work_sum += summaries_9h[s][i]
        working_column_totals_9h.append(round(work_sum, 2))

    working_grand_total_9h = round(sum(working_column_totals_9h), 2)


    # -----------------------------
    # 24H表の合計（既存）
    # -----------------------------
    row_totals_24h = {state: round(sum(summaries_24h[state]), 2) for state in states}

    num_days_24h = len(labels_24h)
    column_totals_24h = []
    
    # --- 24H 稼働時間合計(時間)（自動+手動+完了）---
    working_states = ["自動加工中", "手動加工中", "加工完了"]

    working_column_totals_24h = []
    for i in range(num_days_24h):
        work_sum = 0.0
        for s in working_states:
            if i < len(summaries_24h[s]):
                work_sum += summaries_24h[s][i]
        working_column_totals_24h.append(round(work_sum, 2))

    working_grand_total_24h = round(sum(working_column_totals_24h), 2)

    
    for i in range(num_days_24h):
        day_sum = 0.0
        for state in states:
            if i < len(summaries_24h[state]):
                day_sum += summaries_24h[state][i]
        column_totals_24h.append(round(day_sum, 2))
    grand_total_24h = round(sum(row_totals_24h.values()), 2)

    # -----------------------------
    # 9H 月次棒グラフ（%）＋ 合計（%）折れ線を生成
    # -----------------------------
    # 横軸は 1..月末日
    year = target_month.year
    month = target_month.month
    dd_max = monthrange(year, month)[1]

    # 日->状態別(時間) を作る（データが無い日は0）
    map_9h_hours = {}
    for idx, d in enumerate(labels_9h):
        per_day = {s: 0.0 for s in states}
        for s in states:
            if idx < len(summaries_9h[s]):
                per_day[s] = summaries_9h[s][idx]
        map_9h_hours[d] = per_day

    working_states = ["自動加工中", "手動加工中", "加工完了"]

    # plot 用の配列
    days = list(range(1, dd_max + 1))
    bar_values_pct = {s: [] for s in states}
    working_total_pct = []

    for day in days:
        date_str = f"{year_month}-{day:02d}"
        per_day = map_9h_hours.get(date_str, {s: 0.0 for s in states})

        # 各状態：9hを100%とする
        for s in states:
            bar_values_pct[s].append((per_day[s] / 9.0) * 100.0)

        # 稼働時間合計（%）＝ 自動+手動+完了
        working_h = 0.0
        for s in working_states:
            working_h += per_day[s]
        working_total_pct.append((working_h / 9.0) * 100.0)

    # 画像保存
    os.makedirs("static", exist_ok=True)
    summary9_png = f"{year_month}_summary_9h.png"
    summary9_path = os.path.join("static", summary9_png)

    set_japanese_font()
    plt.figure(figsize=(16, 5))

    # 5本棒（グループ化）
    x = days
    width = 0.15
    offsets = {
        "自動加工中": -2*width,
        "手動加工中": -1*width,
        "加工完了":   0*width,
        "アラーム":   1*width,
        "停止":       2*width,
    }
    colors = {
        "自動加工中": "green",
        "手動加工中": "blue",
        "加工完了":   "yellow",
        "アラーム":   "red",
        "停止":       "gray",
    }

    for s in states:
        xs = [v + offsets[s] for v in x]
        plt.bar(xs, bar_values_pct[s], width=width, label=s, color=colors[s])

    # 合計（%）を折れ線（点付き）
    plt.plot(x, working_total_pct, marker="o", linestyle="-", label="稼働時間合計(%)")

    plt.xticks(days, [str(d) for d in days])
    plt.ylim(0, 100)
    plt.xlabel("日")
    plt.ylabel("稼働率(%)（9H=100%）")
    plt.title(f"{year_month} 定時(8:00-17:00) 稼働率（9Hベース）")
    plt.grid(True, axis="y", linestyle="--", linewidth=0.5)
    plt.legend()
    plt.tight_layout()

    if os.path.exists(summary9_path):
        os.remove(summary9_path)
    plt.savefig(summary9_path)
    plt.close()

    return render_template(
        "month/summary.html",
        year_month=year_month,
        states=states,

        # --- 9H ---
        labels_9h=labels_9h,
        summaries_9h=summaries_9h,
        row_totals_9h=row_totals_9h,
        column_totals_9h=column_totals_9h,
        grand_total_9h=grand_total_9h,
        working_column_totals_9h=working_column_totals_9h,
        working_grand_total_9h=working_grand_total_9h,
        summary9_png=summary9_png,

        # --- 24H（既存）---
        labels=labels_24h,
        summaries=summaries_24h,
        row_totals=row_totals_24h,
        column_totals=column_totals_24h,
        grand_total=grand_total_24h,
        working_column_totals_24h=working_column_totals_24h,
        working_grand_total_24h=working_grand_total_24h,
    )


@app.route("/date/<date>/overview")
def show_date_overview(date):
    try:
        datetime.strptime(date, "%Y-%m-%d")
    except ValueError:
        abort(404)

    items = []
    # 行1（日全体）
    day_img = f"{date}_graph.png"
    generate_graph_image_unified(date_str=date, out_png_path=os.path.join("static", day_img))
    day_durations = summarize_states_full_day_hours(date)
    if day_durations is None:
        abort(404, description=f"{date}.csv が見つかりません。")
    items.append({
        "kind": "day",
        "index": None,
        "info": None,
        "durations": day_durations,
        "image_filename": day_img
    })

    # 品目列（新CSV：状態＋開始/停止×5）
    headers, records, _ = read_hinmoku_csv(date)
    if headers and records:
        for idx, row in enumerate(records, start=1):
            intervals = extract_intervals_from_row(date, row)
            if not intervals:
                continue

            # 状態別集計（時間）
            secs = summarize_states_for_intervals(date, intervals)
            durations_hours = {k: round(v / 3600.0, 2) for k, v in secs.items()}

            # 画像（複数区間）
            img_name = f"{date}_hinmoku_{idx}.png"
            ok = generate_graph_image_for_intervals(date, intervals, os.path.join("static", img_name))
            image_filename = img_name if ok else None

            intervals_str = " / ".join(f"{s.strftime('%H:%M')}-{e.strftime('%H:%M')}" for s, e in intervals)

            items.append({
                "kind": "item",
                "index": idx,
                "info": {
                    "kikai_no": row[0],
                    "seiban": row[1],
                    "tehai_no": row[2],
                    "hinmoku_no": row[3],
                    "hinmoku_name": row[4],
                    "status": (row[8] if len(row) > 8 else ""),
                    "intervals": intervals_str,
                },
                "durations": durations_hours,
                "image_filename": image_filename
            })

    year_month = datetime.strptime(date, "%Y-%m-%d").strftime("%Y-%m")
    return render_template("date/overview.html", date=date, year_month=year_month, items=items)
    
@app.route("/date/<date>/table")
def show_table(date):
    filename = f"{date}.csv"
    filepath = os.path.join(DATA_DIR, filename)
    if not os.path.exists(filepath):
        abort(404)
    rows = []
    with open(filepath, newline='', encoding='utf-8') as f:
        for row in csv.reader(f):
            if len(row) < 5:
                continue
            rows.append({
                "time": row[0],
                "red": row[1],
                "yellow": row[2],
                "green": row[3],
                "current": row[4]
            })
    
    year_month = datetime.strptime(date, "%Y-%m-%d").strftime("%Y-%m")
    
    return render_template("date/sensor_data_list.html", date=date,year_month=year_month, rows=rows)

@app.route("/date/<date>/status")
def show_status_table(date):
    filename = f"{date}.csv"
    filepath = os.path.join(DATA_DIR, filename)
    if not os.path.exists(filepath):
        abort(404)
    rows = []
    with open(filepath, newline='', encoding='utf-8') as f:
        for row in csv.reader(f):
            if len(row) < 5:
                continue
            red, yellow, green, current = float(row[1]), float(row[2]), float(row[3]), float(row[4])
            lights, machine_action, state, color = get_light_status(red, yellow, green, current)
            rows.append({
                "time": row[0],
                "red": lights["red"],
                "yellow": lights["yellow"],
                "green": lights["green"],
                "machine_action": machine_action,
                "state": state,
                "color": color
            })

    year_month = datetime.strptime(date, "%Y-%m-%d").strftime("%Y-%m")
    return render_template("date/status_list.html", date=date,year_month=year_month, rows=rows)

@app.route("/date/<date>/graph")
def show_graph(date):
    image_filename = f"{date}_graph.png"
    image_path = os.path.join("static", image_filename)

    csv_path = os.path.join(DATA_DIR, f"{date}.csv")
    if not os.path.exists(csv_path):
        abort(404)

    # グラフ生成（既存ならスキップ）
    generate_graph_image(date)

    if not os.path.exists(image_path):
        abort(400, description="グラフ画像の生成に失敗しました。")

    year_month = datetime.strptime(date, "%Y-%m-%d").strftime("%Y-%m")

    return render_template("date/graph.html", date=date, year_month=year_month, image_filename=image_filename)

@app.route("/date/<date>/summary")
def show_day_summary(date):
    filepath = os.path.join(DATA_DIR, f"{date}.csv")
    if not os.path.exists(filepath):
        abort(404)

    states = ["自動加工中", "手動加工中", "加工完了", "アラーム", "停止"]
    durations = {state: 0 for state in states}

    with open(filepath, newline='', encoding='utf-8') as f:
        for row in csv.reader(f):
            if len(row) < 5:
                continue
            try:
                r, y, g, c = float(row[1]), float(row[2]), float(row[3]), float(row[4])
                _, _, state, _ = get_light_status(r, y, g, c)
                durations[state] += 60  # assuming 1 minute resolution
            except:
                continue

    for key in durations:
        durations[key] = round(durations[key] / 3600, 2)

    year_month = datetime.strptime(date, "%Y-%m-%d").strftime("%Y-%m")

    return render_template("date/summary.html", date=date, year_month=year_month, durations=durations)

@app.route("/date/<date>/hinmoku")
def show_hinmoku_for_date(date):
    """
    指定日付の品目一覧（行頭に 1..N の行番号列を追加＆リンク化）
    """
    # 日付バリデーション
    try:
        datetime.strptime(date, "%Y-%m-%d")
    except ValueError:
        abort(404)

    headers, records, filename = read_hinmoku_csv(date)
    if not headers or records is None:
        return render_template(
            "date/hinmoku_list.html",
            has_data=False,
            date=date,
            message="品目リストはありません"
        )

    # 先頭に「#」列を追加したヘッダ
    display_headers = ["#"] + headers

    # 行番号を付与（1始まり）。リンク先は /date/<date>/hinmoku/<idx>
    # テンプレ側でリンクを生成するため、ここではインデックスのみ渡す
    indexed_records = []
    for i, row in enumerate(records, start=1):
        indexed_records.append((i, row))  # (行番号, 元の行)

    year_month = datetime.strptime(date, "%Y-%m-%d").strftime("%Y-%m")
    
    return render_template(
        "date/hinmoku_list.html",
        has_data=True,
        date=date,
        year_month=year_month,
        headers=display_headers,
        records=indexed_records,
        filename=filename
    )

@app.route("/date/<date>/hinmoku/<int:hinmokuno>")
def show_hinmoku_graph(date, hinmokuno):
    try:
        datetime.strptime(date, "%Y-%m-%d")
    except ValueError:
        abort(404)

    headers, records, filename = read_hinmoku_csv(date)
    if not headers or not records:
        abort(404, description="品目リストがありません。")

    if hinmokuno < 1 or hinmokuno > len(records):
        abort(404, description="指定の品目番号が範囲外です。")

    row = records[hinmokuno - 1]
    intervals = extract_intervals_from_row(date, row)
    if not intervals:
        abort(400, description="有効な開始/停止区間がありません。")

    image_filename = f"{date}_hinmoku_{hinmokuno}.png"
    image_path = os.path.join("static", image_filename)
    ok = generate_graph_image_for_intervals(date, intervals, image_path)
    if not ok:
        abort(400, description="グラフ画像の生成に失敗しました。対象区間にデータが無い可能性があります。")

    year_month = datetime.strptime(date, "%Y-%m-%d").strftime("%Y-%m")
    return render_template(
        "hinmoku/graph.html",
        date=date,
        year_month=year_month,
        hinmokuno=hinmokuno,
        image_filename=image_filename,
        row=row,
        headers=headers
    )

@app.route("/date/<date>/hinmoku/<int:hinmokuno>/summary")
def show_hinmoku_summary(date, hinmokuno):
    try:
        datetime.strptime(date, "%Y-%m-%d")
    except ValueError:
        abort(404)

    headers, records, filename = read_hinmoku_csv(date)
    if not headers or not records:
        abort(404, description="品目リストがありません。")

    if hinmokuno < 1 or hinmokuno > len(records):
        abort(404, description="指定の品目番号が範囲外です。")

    row = records[hinmokuno - 1]
    intervals = extract_intervals_from_row(date, row)
    if not intervals:
        abort(404, description="有効な開始/停止区間がありません。")

    secs = summarize_states_for_intervals(date, intervals)
    if secs is None:
        abort(404, description=f"{date}.csv が見つかりません。")

    durations_hours = {k: round(v / 3600.0, 2) for k, v in secs.items()}
    interval_info = {
        "intervals": " / ".join(f"{s.strftime('%H:%M')}-{e.strftime('%H:%M')}" for s, e in intervals),
        "clipped_date": date,
        "status": (row[8] if len(row) > 8 else "")
    }

    year_month = datetime.strptime(date, "%Y-%m-%d").strftime("%Y-%m")
    return render_template(
        "hinmoku/summary.html",
        date=date,
        year_month=year_month,
        hinmokuno=hinmokuno,
        headers=headers,
        row=row,
        durations=durations_hours,
        interval=interval_info,
        filename=filename
    )

@app.route("/date/<date>/hinmoku/<int:hinmokuno>/info")
def show_hinmoku_info(date, hinmokuno):
    try:
        datetime.strptime(date, "%Y-%m-%d")
    except ValueError:
        abort(404)

    headers, records, filename = read_hinmoku_csv(date)
    if not headers or not records:
        abort(404, description="品目リストがありません。")

    if hinmokuno < 1 or hinmokuno > len(records):
        abort(404, description="指定の品目番号が範囲外です。")

    row = records[hinmokuno - 1]
    intervals = extract_intervals_from_row(date, row)
    if not intervals:
        abort(404, description="有効な開始/停止区間がありません。")

    secs = summarize_states_for_intervals(date, intervals)
    if secs is None:
        abort(404, description=f"{date}.csv が見つかりません。")

    durations_hours = {k: round(v / 3600.0, 2) for k, v in secs.items()}
    interval_info = {
        "intervals": " / ".join(f"{s.strftime('%H:%M')}-{e.strftime('%H:%M')}" for s, e in intervals),
        "clipped_date": date,
        "status": (row[8] if len(row) > 8 else "")
    }

    year_month = datetime.strptime(date, "%Y-%m-%d").strftime("%Y-%m")
    return render_template(
        "hinmoku/info.html",
        date=date,
        year_month=year_month,
        hinmokuno=hinmokuno,
        headers=headers,
        row=row,
        durations=durations_hours,
        interval=interval_info,
        filename=filename
    )

if __name__ == "__main__":
    app.run(debug=True, host="0.0.0.0", port=5000)
