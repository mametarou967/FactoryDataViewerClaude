from flask import Flask, render_template, abort, send_file, redirect, url_for, jsonify, request
import csv
import os
import struct
import zlib
from datetime import datetime, timedelta, time
import time as _time
import threading
import queue
import uuid

try:
    import yaml
    HAS_YAML = True
except ImportError:
    HAS_YAML = False
    print("[config] pyyaml がインストールされていません。config.yaml は読み込まれません。")

try:
    import serial
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False
    print("[E220] pyserial がインストールされていません。ポーリングを無効化します。")

import matplotlib
matplotlib.use('Agg')
from matplotlib import pyplot as plt
import matplotlib.font_manager as fm
from collections import defaultdict
from calendar import monthrange


app = Flask(__name__)
DATA_DIR = "data/sensor"
HINMOKU_DIR = "data/hinmoku"

# デフォルト閾値（config.yaml の機械設定で上書き可）
THRESHOLDS = {
    "red": 200,
    "yellow": 200,
    "green": 200
}
CURRENT_THRESHOLD = 3.0


# ===== 設定ロード =====

def _parse_addr(v):
    """YAMLが整数またはhex文字列のどちらで返してもアドレスを正しく解釈する。
    yaml.safe_load は 0x0101 を int(257) として読むため、すでに int の場合はそのまま使う。
    """
    if isinstance(v, int):
        return v
    return int(str(v), 16)


def load_config(path=None):
    if path is None:
        path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'config.yaml')
    if not HAS_YAML or not os.path.exists(path):
        return {
            'serial_port': '/dev/ttyUSB0',
            'serial_baud': 9600,
            'gw_channel': 2,
            'gw_addr': 0x0000,
            'poll_interval_sec': 60,
            'machines': []
        }
    try:
        with open(path) as f:
            cfg = yaml.safe_load(f)
        for m in cfg.get('machines', []):
            m['patlite_addr'] = _parse_addr(m['patlite_addr'])
            m['current_addr']  = _parse_addr(m['current_addr'])
        cfg['gw_addr'] = _parse_addr(cfg['gw_addr'])
        return cfg
    except Exception as e:
        print(f"[config] 設定ファイル読み込みエラー: {e}")
        return {
            'serial_port': '/dev/ttyUSB0',
            'serial_baud': 9600,
            'gw_channel': 2,
            'gw_addr': 0x0000,
            'poll_interval_sec': 60,
            'machines': []
        }


config = load_config()


# ===== E220ドライバ =====

def e220_send(ser, dest_addr, channel, cmd_char):
    pkt = bytes([(dest_addr >> 8) & 0xFF, dest_addr & 0xFF,
                 channel, ord(cmd_char), 0x0D, 0x0A])
    ser.reset_input_buffer()
    ser.write(pkt)


def e220_recv(ser, timeout_sec=2.5):
    deadline = _time.time() + timeout_sec
    buf = bytearray()
    while _time.time() < deadline:
        if ser.in_waiting:
            buf += ser.read(ser.in_waiting)
            if len(buf) >= 3 and buf[-2] == 0x0D and buf[-1] == 0x0A:
                return bytes(buf)
        _time.sleep(0.005)
    return None


def crc16_ccitt(data: bytes) -> int:
    """CRC16-CCITT (poly=0x1021, init=0xFFFF)"""
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = (crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1
    return crc & 0xFFFF


def e220_send_payload(ser, addr: int, ch: int, payload: bytes):
    """任意バイト列を E220 固定アドレスモードで送信（最大200バイト）"""
    addrH = (addr >> 8) & 0xFF
    addrL = addr & 0xFF
    packet = bytes([addrH, addrL, ch]) + payload
    ser.write(packet)
    # AUX HIGH 待ち相当（200msポーリング）
    deadline = _time.time() + 2.0
    while _time.time() < deadline:
        _time.sleep(0.02)


def parse_patlite(data):
    # [ADDR_H][ADDR_L]['P'][R_H][R_L][Y_H][Y_L][G_H][G_L][CR][LF]
    if data and len(data) >= 9 and data[2] == ord('P'):
        return (
            (data[3] << 8) | data[4],
            (data[5] << 8) | data[6],
            (data[7] << 8) | data[8]
        )
    return None


def parse_current(data):
    # [ADDR_H][ADDR_L]['C'][CUR_H][CUR_L][CR][LF]
    if data and len(data) >= 5 and data[2] == ord('C'):
        return ((data[3] << 8) | data[4]) / 100.0
    return None


# ===== CSV書き込み =====

def write_sensor_csv(machine_name, red, yellow, green, current):
    now  = datetime.now()
    dirp = os.path.join(DATA_DIR, machine_name)
    os.makedirs(dirp, exist_ok=True)
    path = os.path.join(dirp, f"{now.strftime('%Y-%m-%d')}.csv")
    with open(path, 'a', newline='', encoding='utf-8') as f:
        csv.writer(f).writerow([now.strftime('%H:%M:%S'), red, yellow, green, current])


# ===== ポーリングスレッド =====

g_cmd_q     = queue.Queue()
g_results   = {}               # {req_id: {'status': 'pending'|'done', ...}}
g_res_lock  = threading.Lock()
g_maint_event = threading.Event()  # Flaskがコマンドを積んだらセット → polling_loopが早期起床

# ===== OTA グローバル =====
serial_lock  = threading.Lock()   # ポーリングスレッドとOTAワーカーの排他制御
g_ota_jobs   = {}                  # {job_id: {status, progress, message, fw_bytes, ...}}
g_ota_lock   = threading.Lock()
g_serial_obj = None                # polling_loop が開いているシリアルオブジェクト（OTA共用）


def poll_machine(ser, machine):
    ch = config['gw_channel']
    red = yellow = green = current = None
    e220_send(ser, machine['patlite_addr'], ch, 'P')
    lux = parse_patlite(e220_recv(ser))
    if lux:
        red, yellow, green = lux
    e220_send(ser, machine['current_addr'], ch, 'C')
    amp = parse_current(e220_recv(ser))
    if amp is not None:
        current = amp
    write_sensor_csv(machine['name'],
                     red or 0.0, yellow or 0.0,
                     green or 0.0, current or 0.0)


def _handle_maint(ser, req):
    req_id = req['req_id']
    cmd    = req['cmd']   # 'K' / 'V' / 'H'
    addr   = req['addr']  # int
    ch     = config['gw_channel']

    e220_send(ser, addr, ch, cmd)
    data = e220_recv(ser)
    _time.sleep(0.3)  # E220が受信待ち状態から抜けるのを待つ（連続送信時のタイムアウト誤検知防止）

    if data is None:
        result = {'ok': False, 'error': 'timeout'}
    elif len(data) >= 3 and data[2] == ord('E'):
        code = data[3] if len(data) > 3 else 0
        result = {'ok': False, 'error': f'edge error 0x{code:02X}'}
    elif cmd == 'K':
        result = {'ok': True}
    elif cmd == 'V':
        result = ({'ok': True, 'version': f"{data[3]}.{data[4]}.{data[5]}"}
                  if len(data) >= 6 else {'ok': False, 'error': 'short response'})
    elif cmd == 'H':
        if len(data) >= 9:
            dip = data[3]
            result = {'ok': True,
                      'dip': f"0x{dip:02X}",
                      'dip_patlite': bool(dip & 0x01),
                      'dip_current': bool(dip & 0x02),
                      'e220_addr': f"0x{data[4]:02X}{data[5]:02X}",
                      'e220_ch': data[6],
                      'air_rate': f"0x{data[7]:02X}",
                      'tx_power': f"0x{data[8]:02X}"}
        else:
            result = {'ok': False, 'error': 'short response'}
    else:
        result = {'ok': False, 'error': 'unknown cmd'}

    with g_res_lock:
        g_results[req_id] = {'status': 'done', 'result': result,
                              'machine': req['machine'], 'unit': req['unit'],
                              'cmd': cmd, 'ts': _time.time()}


def polling_loop():
    global g_serial_obj
    if not HAS_SERIAL:
        print("[E220] pyserial がインストールされていません。ポーリングを無効化します。")
        return
    if not config.get('machines'):
        print("[E220] machines が設定されていません。ポーリングを無効化します。")
        return
    while True:
        try:
            with serial.Serial(config['serial_port'],
                               config['serial_baud'], timeout=0.1) as ser:
                print(f"[E220] {config['serial_port']} 接続")
                g_serial_obj = ser
                while True:
                    # OTAワーカーが動作中はポーリングをスキップ
                    if serial_lock.locked():
                        _time.sleep(1.0)
                        continue
                    t0 = _time.time()
                    with serial_lock:
                        # メンテコマンドを優先処理（ポーリング前）
                        while True:
                            try:
                                _handle_maint(ser, g_cmd_q.get_nowait())
                            except queue.Empty:
                                break
                        # 通常ポーリング
                        for machine in config['machines']:
                            try:
                                poll_machine(ser, machine)
                            except Exception as e:
                                print(f"[E220] poll_machine({machine['name']}) エラー: {e}")
                        # ポーリング中に積まれたコマンドも処理
                        while True:
                            try:
                                _handle_maint(ser, g_cmd_q.get_nowait())
                            except queue.Empty:
                                break
                    # 残り時間スリープ（1秒ごとにキューを確認して早期起床）
                    remaining = config['poll_interval_sec'] - (_time.time() - t0)
                    deadline = _time.time() + max(0, remaining)
                    while _time.time() < deadline:
                        g_maint_event.clear()
                        g_maint_event.wait(timeout=min(1.0, max(0, deadline - _time.time())))
                        if not g_cmd_q.empty():
                            break
        except Exception as e:
            print(f"[E220] {e}  5秒後に再接続…")
            g_serial_obj = None
            _time.sleep(5)


threading.Thread(target=polling_loop, daemon=True).start()


# ===== 機械設定ヘルパー =====

def _get_machine_or_404(machine_name):
    for m in config['machines']:
        if m['name'] == machine_name:
            return m
    abort(404, description=f"機械 '{machine_name}' が見つかりません。")


def _first_machine_name():
    """後方互換リダイレクト用の最初の機械名"""
    if config['machines']:
        return config['machines'][0]['name']
    return 'default'


# ===== 点灯・状態判定 =====

def get_light_status(red, yellow, green, current,
                     thresholds=None, current_threshold=None):
    if thresholds is None:
        thresholds = THRESHOLDS
    if current_threshold is None:
        current_threshold = CURRENT_THRESHOLD

    def is_on(color, value):
        return value >= thresholds[color]

    status = {
        "red":    "点灯" if is_on("red",    red)    else "消灯",
        "yellow": "点灯" if is_on("yellow", yellow) else "消灯",
        "green":  "点灯" if is_on("green",  green)  else "消灯"
    }

    machine_action = "加工中" if current >= current_threshold else "加工なし"

    state, color = "停止", "gray"
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


# ===== 最新データ取得 =====

def get_latest_data(machine_name):
    dirpath = os.path.join(DATA_DIR, machine_name)
    if not os.path.isdir(dirpath):
        return None
    now = datetime.now()
    threshold = now - timedelta(minutes=5)
    for filename in sorted(os.listdir(dirpath), reverse=True):
        if not filename.endswith(".csv"):
            continue
        filepath = os.path.join(dirpath, filename)
        with open(filepath, newline='', encoding='utf-8') as f:
            reader = csv.reader(f)
            for row in reversed(list(reader)):
                try:
                    row_time = datetime.strptime(
                        f"{filename[:-4]} {row[0]}", "%Y-%m-%d %H:%M:%S")
                except Exception:
                    continue
                if threshold <= row_time <= now:
                    return {
                        "time":      row[0],
                        "red":       float(row[1]),
                        "yellow":    float(row[2]),
                        "green":     float(row[3]),
                        "current":   float(row[4]),
                        "timestamp": row_time.strftime("%Y-%m-%d %H:%M:%S")
                    }
    return None


def read_hinmoku_csv(date_str, hinmoku_prefix=None):
    """
    品目CSV: data/hinmoku/<prefix>_YYYYMMDD.csv を読み、(headers, rows, filename) を返す。
    hinmoku_prefix が None のときは "A214" を使う（後方互換）。
    文字コードは cp932 優先、失敗時に utf-8 にフォールバック。
    """
    if hinmoku_prefix is None:
        hinmoku_prefix = "A214"
    yyyymmdd = datetime.strptime(date_str, "%Y-%m-%d").strftime("%Y%m%d")
    expected_name = f"{hinmoku_prefix}_{yyyymmdd}.csv"
    dirpath  = HINMOKU_DIR
    filepath = os.path.join(dirpath, expected_name)

    if not os.path.isdir(dirpath) or not os.path.exists(filepath):
        return None, None, expected_name

    rows = []
    for enc in ["cp932", "utf-8"]:
        try:
            with open(filepath, newline="", encoding=enc) as f:
                reader = csv.reader(f)
                for row in reader:
                    if row:
                        rows.append(row)
            break
        except Exception:
            rows = []
            continue

    if not rows:
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


def _load_minute_colors(date_str, machine_name, start_dt=None, end_dt=None,
                        include_gray=True, thresholds=None, current_threshold=None):
    """
    data/sensor/<machine_name>/<date_str>.csv を読み、分単位の色辞書 {datetime: color} を返す。
    """
    csv_path = os.path.join(DATA_DIR, machine_name, f"{date_str}.csv")
    if not os.path.exists(csv_path):
        return None

    day_start, day_end = _day_range(date_str)
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
            if not (s <= t < e):
                continue
            try:
                r, y, g, c = float(row[1]), float(row[2]), float(row[3]), float(row[4])
                _, _, _, color = get_light_status(r, y, g, c,
                                                  thresholds=thresholds,
                                                  current_threshold=current_threshold)
                if color == "gray" and not include_gray:
                    continue
                minute_color[t] = color
            except Exception:
                continue

    return minute_color


def _render_day_timeline(date_str, minute_color, out_png_path, title):
    """1日横棒を描画。minute_color に入っている分だけ色を塗る。"""
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
    machine_name,
    start_dt=None,
    end_dt=None,
    out_png_path=None,
    include_gray=True,
    skip_if_up_to_date=True,
    thresholds=None,
    current_threshold=None,
):
    """
    1本化された描画関数。
    - 日全体: start_dt/end_dt を渡さない
    - 区間のみ: start_dt/end_dt を渡す（[start_dt, end_dt)）
    - out_png_path 未指定時はデイリーの既定パスを使う
    """
    csv_path = os.path.join(DATA_DIR, machine_name, f"{date_str}.csv")
    if not os.path.exists(csv_path):
        return False

    if out_png_path is None:
        image_filename = f"{machine_name}_{date_str}_graph.png"
        out_png_path = os.path.join("static", image_filename)

    is_full_day = (start_dt is None and end_dt is None)
    if skip_if_up_to_date and is_full_day and os.path.exists(out_png_path):
        if os.path.getmtime(csv_path) <= os.path.getmtime(out_png_path):
            return True

    minute_color = _load_minute_colors(
        date_str, machine_name,
        start_dt=start_dt, end_dt=end_dt,
        include_gray=include_gray,
        thresholds=thresholds, current_threshold=current_threshold
    )
    if not minute_color:
        return False

    if is_full_day:
        title = f"{machine_name} {date_str} 状態推移グラフ"
    else:
        s = start_dt.strftime("%H:%M")
        e = end_dt.strftime("%H:%M")
        title = f"{machine_name} {date_str} 品目時間帯グラフ（{s}〜{e}）"

    _render_day_timeline(date_str, minute_color, out_png_path, title)
    return True


def generate_graph_image(date, machine_name):
    """互換ラッパー"""
    return generate_graph_image_unified(date_str=date, machine_name=machine_name)


def generate_graph_image_for_interval(date_str, start_dt, end_dt, out_png_path, machine_name):
    """互換ラッパー"""
    return generate_graph_image_unified(
        date_str=date_str, machine_name=machine_name,
        start_dt=start_dt, end_dt=end_dt,
        out_png_path=out_png_path, include_gray=True,
        skip_if_up_to_date=False,
    )


def generate_graph_image_for_intervals(date_str, intervals, out_png_path, machine_name):
    """複数区間の合成グラフを1枚に描画。"""
    if not intervals:
        return False

    merged = {}
    for s_dt, e_dt in intervals:
        mc = _load_minute_colors(date_str, machine_name,
                                 start_dt=s_dt, end_dt=e_dt, include_gray=True)
        if not mc:
            continue
        merged.update(mc)

    if not merged:
        return False

    s_label = intervals[0][0].strftime("%H:%M")
    e_label = intervals[-1][1].strftime("%H:%M")
    title = (f"{machine_name} {date_str} 品目時間帯グラフ"
             f"（{s_label}〜{e_label}／{len(intervals)}区間）")

    _render_day_timeline(date_str, merged, out_png_path, title)
    return True


# --- 柔軟な日時パーサ（秒あり/なしを許容） ---
def parse_flexible_dt(s):
    s = s.strip()
    fmts = [
        "%Y/%m/%d %H:%M:%S",
        "%Y/%m/%d %H:%M",
        "%Y/%-m/%-d %-H:%-M:%-S",
        "%Y/%-m/%-d %-H:%-M",
    ]
    for fmt in fmts:
        try:
            return datetime.strptime(s, fmt)
        except Exception:
            continue
    try:
        parts = s.replace("/", " ").replace(":", " ").split()
        if len(parts) >= 5:
            Y, M, D, h, m = parts[:5]
            sec = parts[5] if len(parts) >= 6 else "00"
            canon = (f"{int(Y):04d}/{int(M):02d}/{int(D):02d} "
                     f"{int(h):02d}:{int(m):02d}:{int(sec):02d}")
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
    for i in range(5):
        s_idx = 9 + 2*i
        e_idx = 10 + 2*i
        if len(row) <= s_idx:
            break
        s_raw = (row[s_idx] or "").strip()
        e_raw = (row[e_idx] if len(row) > e_idx else "")
        e_raw = (e_raw or "").strip()

        if not s_raw:
            continue

        try:
            s_dt = parse_flexible_dt(s_raw)
        except Exception:
            continue

        e_dt = None
        if e_raw:
            try:
                e_dt = parse_flexible_dt(e_raw)
            except Exception:
                e_dt = None
        if e_dt is None:
            e_dt = min(now, day_end) if base_date == now.date() else day_end

        s_dt = max(s_dt, day_start)
        e_dt = min(e_dt, day_end)

        if e_dt <= s_dt:
            e_dt = min(s_dt + timedelta(minutes=1), day_end)

        if s_dt < e_dt:
            intervals.append((s_dt, e_dt))

    intervals.sort(key=lambda t: t[0])
    return intervals


def resolve_item_interval(date_str, start_raw, end_raw):
    """品目CSVの着手・完了文字列から区間 [start_dt, end_dt) を決める。"""
    base_date = datetime.strptime(date_str, "%Y-%m-%d").date()
    day_start = datetime.combine(base_date, time(0, 0, 0))
    day_end   = datetime.combine(base_date, time(23, 59, 59))

    start_dt = parse_flexible_dt((start_raw or "").strip())

    end_dt = None
    end_raw = (end_raw or "").strip()
    if end_raw:
        try:
            end_dt = parse_flexible_dt(end_raw)
        except Exception:
            end_dt = None

    if end_dt is None:
        now = datetime.now()
        if base_date == now.date():
            end_dt = min(now, day_end)
        else:
            end_dt = day_end

    if end_dt <= start_dt:
        end_dt = min(start_dt + timedelta(minutes=1), day_end)

    return start_dt, end_dt


def get_current_processing_items(now=None, machine_config=None):
    if now is None:
        now = datetime.now()
    today_str = now.strftime("%Y-%m-%d")
    hinmoku_prefix = machine_config.get('hinmoku_prefix') if machine_config else None

    headers, records, expected_name = read_hinmoku_csv(today_str,
                                                       hinmoku_prefix=hinmoku_prefix)
    result = {"has_csv": False, "expected": expected_name, "items": []}
    if not headers or records is None:
        return result
    result["has_csv"] = True

    for idx, row in enumerate(records, start=1):
        intervals = extract_intervals_from_row(today_str, row)
        if not intervals:
            continue

        in_any = any(s <= now < e for s, e in intervals)
        if not in_any:
            continue

        intervals_str = " / ".join(
            f"{s.strftime('%H:%M')}-{e.strftime('%H:%M')}" for s, e in intervals)

        info = {
            "kikai_no":     row[0],
            "seiban":       row[1],
            "tehai_no":     row[2],
            "hinmoku_no":   row[3],
            "hinmoku_name": row[4],
            "intervals":    intervals_str,
            "status":       (row[8] if len(row) > 8 else "")
        }
        result["items"].append({"index": idx, "info": info})

    return result


# --- 区間限定の状態別集計ユーティリティ ---

def summarize_states_for_interval(date_str, start_dt, end_dt, machine_name,
                                   thresholds=None, current_threshold=None):
    csv_path = os.path.join(DATA_DIR, machine_name, f"{date_str}.csv")
    if not os.path.exists(csv_path):
        return None

    base_date = datetime.strptime(date_str, "%Y-%m-%d")
    day_start = datetime.combine(base_date, datetime.strptime("00:00:00", "%H:%M:%S").time())
    day_end   = day_start + timedelta(days=1)

    s = max(start_dt, day_start)
    e = min(end_dt,   day_end)
    if not (s < e):
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
                continue
            if not (s <= t < e):
                continue
            try:
                r, y, g, c = float(row[1]), float(row[2]), float(row[3]), float(row[4])
                _, _, state, _ = get_light_status(r, y, g, c,
                                                  thresholds=thresholds,
                                                  current_threshold=current_threshold)
                secs[state] += 60
            except Exception:
                continue

    return secs


def summarize_states_for_intervals(date_str, intervals, machine_name,
                                    thresholds=None, current_threshold=None):
    if not intervals:
        return {k: 0 for k in ["自動加工中", "手動加工中", "加工完了", "アラーム", "停止"]}
    total = {k: 0 for k in ["自動加工中", "手動加工中", "加工完了", "アラーム", "停止"]}
    for s_dt, e_dt in intervals:
        secs = summarize_states_for_interval(date_str, s_dt, e_dt, machine_name,
                                              thresholds=thresholds,
                                              current_threshold=current_threshold)
        if secs is None:
            continue
        for k, v in secs.items():
            total[k] += v
    return total


def summarize_states_full_day_hours(date_str, machine_name,
                                     thresholds=None, current_threshold=None):
    csv_path = os.path.join(DATA_DIR, machine_name, f"{date_str}.csv")
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
                _, _, state, _ = get_light_status(r, y, g, c,
                                                  thresholds=thresholds,
                                                  current_threshold=current_threshold)
                secs[state] += 60
            except Exception:
                continue
    return {k: round(v / 3600.0, 2) for k, v in secs.items()}


# ===== Flask Routes =====

@app.route("/")
def index():
    now = datetime.now()
    today_str = now.strftime("%Y-%m-%d")

    # --- 各機械の現在状態 ---
    machine_statuses = []
    for m in config['machines']:
        machine_name      = m['name']
        thresholds        = m.get('patlite_thresholds', THRESHOLDS)
        curr_thresh       = m.get('current_threshold', CURRENT_THRESHOLD)
        latest = get_latest_data(machine_name)

        status_summary = None
        status_debug   = None
        if latest:
            lights, machine_action, state, color = get_light_status(
                latest["red"], latest["yellow"], latest["green"], latest["current"],
                thresholds=thresholds, current_threshold=curr_thresh
            )
            status_summary = {
                "state":     state,
                "color":     color,
                "current":   latest["current"],
                "timestamp": latest["timestamp"]
            }
            status_debug = {
                **lights,
                "current":   latest["current"],
                "timestamp": latest["timestamp"]
            }
        machine_statuses.append({
            "name":              machine_name,
            "status_summary":    status_summary,
            "status_debug":      status_debug,
            "thresholds":        thresholds,
            "current_threshold": curr_thresh,
        })

    # --- 現在の加工状況（最初の機械） ---
    first_machine_config = config['machines'][0] if config['machines'] else None
    first_machine_name   = first_machine_config['name'] if first_machine_config else None
    current_work = get_current_processing_items(now=now, machine_config=first_machine_config)

    # --- 年度カレンダー（最初の機械） ---
    if now.month >= 4:
        fiscal_start = datetime(now.year, 4, 1)
    else:
        fiscal_start = datetime(now.year - 1, 4, 1)
    fiscal_end = fiscal_start.replace(year=fiscal_start.year + 1) - timedelta(days=1)

    existing_days   = set()
    existing_months = set()
    if first_machine_name:
        cal_dir = os.path.join(DATA_DIR, first_machine_name)
        if os.path.isdir(cal_dir):
            for fname in os.listdir(cal_dir):
                if fname.endswith(".csv"):
                    try:
                        date_obj = datetime.strptime(fname[:-4], "%Y-%m-%d")
                        if fiscal_start <= date_obj <= fiscal_end:
                            existing_days.add(date_obj.date())
                            existing_months.add(date_obj.strftime("%Y-%m"))
                    except Exception:
                        continue

    calendar = {}
    current_day = fiscal_start
    while current_day <= fiscal_end:
        ym = current_day.strftime("%Y-%m")
        if ym not in calendar:
            calendar[ym] = {
                "month_link": ym in existing_months,
                "weeks": [[]]
            }
        week = calendar[ym]["weeks"][-1]
        if len(week) == 0 and current_day.weekday() != 0:
            week.extend([None] * current_day.weekday())
        week.append({
            "day":  current_day.day,
            "date": current_day.strftime("%Y-%m-%d"),
            "link": current_day.date() in existing_days
        })
        if current_day.weekday() == 6:
            calendar[ym]["weeks"].append([])
        current_day += timedelta(days=1)

    return render_template(
        "index.html",
        machine_statuses=machine_statuses,
        first_machine_name=first_machine_name,
        thresholds=THRESHOLDS,
        current_threshold=CURRENT_THRESHOLD,
        calendar=calendar,
        current_work=current_work,
        today=today_str
    )


# ===== /machine/<name>/month/<ym>/ =====

@app.route("/machine/<machine_name>/month/<year_month>/overview")
def show_month_overview(machine_name, year_month):
    machine = _get_machine_or_404(machine_name)
    thresholds    = machine.get('patlite_thresholds', THRESHOLDS)
    curr_thresh   = machine.get('current_threshold', CURRENT_THRESHOLD)
    try:
        target_month = datetime.strptime(year_month, "%Y-%m")
    except ValueError:
        abort(404)

    year   = target_month.year
    month  = target_month.month
    dd_max = monthrange(year, month)[1]

    items = []
    for day in range(1, dd_max + 1):
        date_str = f"{year_month}-{day:02d}"
        csv_path = os.path.join(DATA_DIR, machine_name, f"{date_str}.csv")

        durations      = None
        image_filename = None

        if os.path.exists(csv_path):
            durations = summarize_states_full_day_hours(date_str, machine_name,
                                                        thresholds=thresholds,
                                                        current_threshold=curr_thresh)
            generate_graph_image(date_str, machine_name)
            image_filename = f"{machine_name}_{date_str}_graph.png"

        items.append({
            "date":           date_str,
            "durations":      durations,
            "image_filename": image_filename
        })

    return render_template("month/overview.html",
                           machine_name=machine_name,
                           year_month=year_month, items=items)


@app.route("/machine/<machine_name>/month/<year_month>/graph")
def show_month_graph(machine_name, year_month):
    machine = _get_machine_or_404(machine_name)
    try:
        month_date = datetime.strptime(year_month, "%Y-%m")
    except Exception:
        abort(404)

    year  = month_date.year
    month = month_date.month
    day   = 1
    images = []

    while True:
        try:
            current_date = datetime(year, month, day)
        except Exception:
            break
        date_str = current_date.strftime("%Y-%m-%d")
        csv_path = os.path.join(DATA_DIR, machine_name, f"{date_str}.csv")
        image_filename = f"{machine_name}_{date_str}_graph.png"

        if os.path.exists(csv_path):
            generate_graph_image(date_str, machine_name)
            images.append({"date": date_str, "image_filename": image_filename})
        day += 1

    if not images:
        abort(404)

    return render_template("month/graph.html",
                           machine_name=machine_name,
                           year_month=year_month, images=images)


@app.route("/machine/<machine_name>/month/<year_month>/summary")
def show_month_summary(machine_name, year_month):
    machine = _get_machine_or_404(machine_name)
    thresholds  = machine.get('patlite_thresholds', THRESHOLDS)
    curr_thresh = machine.get('current_threshold', CURRENT_THRESHOLD)
    try:
        target_month = datetime.strptime(year_month, "%Y-%m")
    except ValueError:
        abort(404)

    states = ["自動加工中", "手動加工中", "加工完了", "アラーム", "停止"]

    summaries_9h  = {state: [] for state in states}
    labels_9h     = []
    summaries_24h = {state: [] for state in states}
    labels_24h    = []

    machine_dir = os.path.join(DATA_DIR, machine_name)
    if not os.path.isdir(machine_dir):
        abort(404, description="指定された機械のデータが見つかりませんでした")

    for fname in sorted(os.listdir(machine_dir)):
        if not fname.endswith(".csv"):
            continue
        try:
            date_obj = datetime.strptime(fname[:-4], "%Y-%m-%d")
        except ValueError:
            continue
        if date_obj.strftime("%Y-%m") != year_month:
            continue

        date_str = date_obj.strftime("%Y-%m-%d")
        filepath = os.path.join(machine_dir, fname)

        # 9H（8:00-17:00）
        start_dt = datetime.combine(date_obj.date(), time(8, 0, 0))
        end_dt   = datetime.combine(date_obj.date(), time(17, 0, 0))
        secs_9h  = summarize_states_for_interval(date_str, start_dt, end_dt, machine_name,
                                                  thresholds=thresholds,
                                                  current_threshold=curr_thresh)
        if secs_9h is not None:
            labels_9h.append(date_str)
            for state in states:
                summaries_9h[state].append(round(secs_9h.get(state, 0) / 3600.0, 2))

        # 24H
        labels_24h.append(date_str)
        durations_sec = {state: 0 for state in states}
        with open(filepath, newline='', encoding='utf-8') as f:
            for row in csv.reader(f):
                if len(row) < 5:
                    continue
                try:
                    r, y, g, c = float(row[1]), float(row[2]), float(row[3]), float(row[4])
                    _, _, state, _ = get_light_status(r, y, g, c,
                                                      thresholds=thresholds,
                                                      current_threshold=curr_thresh)
                    durations_sec[state] += 60
                except Exception:
                    continue
        for state in states:
            summaries_24h[state].append(round(durations_sec[state] / 3600.0, 2))

    if not labels_24h and not labels_9h:
        abort(404, description="指定された月にデータが見つかりませんでした")

    # 9H集計
    row_totals_9h    = {state: round(sum(summaries_9h[state]), 2) for state in states}
    num_days_9h      = len(labels_9h)
    working_states   = ["自動加工中", "手動加工中", "加工完了"]
    column_totals_9h = []
    for i in range(num_days_9h):
        day_sum = sum(summaries_9h[s][i] for s in states if i < len(summaries_9h[s]))
        column_totals_9h.append(round(day_sum, 2))
    grand_total_9h = round(sum(row_totals_9h.values()), 2)

    working_column_totals_9h = []
    for i in range(num_days_9h):
        work_sum = sum(summaries_9h[s][i] for s in working_states if i < len(summaries_9h[s]))
        working_column_totals_9h.append(round(work_sum, 2))
    working_grand_total_9h = round(sum(working_column_totals_9h), 2)

    # 24H集計
    row_totals_24h    = {state: round(sum(summaries_24h[state]), 2) for state in states}
    num_days_24h      = len(labels_24h)
    column_totals_24h = []
    for i in range(num_days_24h):
        day_sum = sum(summaries_24h[s][i] for s in states if i < len(summaries_24h[s]))
        column_totals_24h.append(round(day_sum, 2))
    grand_total_24h = round(sum(row_totals_24h.values()), 2)

    working_column_totals_24h = []
    for i in range(num_days_24h):
        work_sum = sum(summaries_24h[s][i] for s in working_states if i < len(summaries_24h[s]))
        working_column_totals_24h.append(round(work_sum, 2))
    working_grand_total_24h = round(sum(working_column_totals_24h), 2)

    # 9H月次棒グラフ
    year   = target_month.year
    month  = target_month.month
    dd_max = monthrange(year, month)[1]

    map_9h_hours = {}
    for idx, d in enumerate(labels_9h):
        per_day = {s: 0.0 for s in states}
        for s in states:
            if idx < len(summaries_9h[s]):
                per_day[s] = summaries_9h[s][idx]
        map_9h_hours[d] = per_day

    days              = list(range(1, dd_max + 1))
    bar_values_pct    = {s: [] for s in states}
    working_total_pct = []

    for day in days:
        date_str = f"{year_month}-{day:02d}"
        per_day  = map_9h_hours.get(date_str, {s: 0.0 for s in states})
        for s in states:
            bar_values_pct[s].append((per_day[s] / 9.0) * 100.0)
        working_h = sum(per_day[s] for s in working_states)
        working_total_pct.append((working_h / 9.0) * 100.0)

    os.makedirs("static", exist_ok=True)
    summary9_png  = f"{machine_name}_{year_month}_summary_9h.png"
    summary9_path = os.path.join("static", summary9_png)

    set_japanese_font()
    plt.figure(figsize=(16, 5))

    x      = days
    width  = 0.15
    offsets = {
        "自動加工中": -2*width,
        "手動加工中": -1*width,
        "加工完了":    0*width,
        "アラーム":    1*width,
        "停止":        2*width,
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

    plt.plot(x, working_total_pct, marker="o", linestyle="-", label="稼働時間合計(%)")
    plt.xticks(days, [str(d) for d in days])
    plt.ylim(0, 100)
    plt.xlabel("日")
    plt.ylabel("稼働率(%)（9H=100%）")
    plt.title(f"{machine_name} {year_month} 定時(8:00-17:00) 稼働率（9Hベース）")
    plt.grid(True, axis="y", linestyle="--", linewidth=0.5)
    plt.legend()
    plt.tight_layout()

    if os.path.exists(summary9_path):
        os.remove(summary9_path)
    plt.savefig(summary9_path)
    plt.close()

    return render_template(
        "month/summary.html",
        machine_name=machine_name,
        year_month=year_month,
        states=states,
        labels_9h=labels_9h,
        summaries_9h=summaries_9h,
        row_totals_9h=row_totals_9h,
        column_totals_9h=column_totals_9h,
        grand_total_9h=grand_total_9h,
        working_column_totals_9h=working_column_totals_9h,
        working_grand_total_9h=working_grand_total_9h,
        summary9_png=summary9_png,
        labels=labels_24h,
        summaries=summaries_24h,
        row_totals=row_totals_24h,
        column_totals=column_totals_24h,
        grand_total=grand_total_24h,
        working_column_totals_24h=working_column_totals_24h,
        working_grand_total_24h=working_grand_total_24h,
    )


# ===== /machine/<name>/date/<date>/ =====

@app.route("/machine/<machine_name>/date/<date>/overview")
def show_date_overview(machine_name, date):
    machine = _get_machine_or_404(machine_name)
    thresholds  = machine.get('patlite_thresholds', THRESHOLDS)
    curr_thresh = machine.get('current_threshold', CURRENT_THRESHOLD)
    hinmoku_prefix = machine.get('hinmoku_prefix')
    try:
        datetime.strptime(date, "%Y-%m-%d")
    except ValueError:
        abort(404)

    items = []
    day_img = f"{machine_name}_{date}_graph.png"
    generate_graph_image_unified(date_str=date, machine_name=machine_name,
                                 out_png_path=os.path.join("static", day_img),
                                 thresholds=thresholds, current_threshold=curr_thresh)
    day_durations = summarize_states_full_day_hours(date, machine_name,
                                                    thresholds=thresholds,
                                                    current_threshold=curr_thresh)
    if day_durations is None:
        abort(404, description=f"{date}.csv が見つかりません。")
    items.append({
        "kind": "day", "index": None, "info": None,
        "durations": day_durations, "image_filename": day_img
    })

    headers, records, _ = read_hinmoku_csv(date, hinmoku_prefix=hinmoku_prefix)
    if headers and records:
        for idx, row in enumerate(records, start=1):
            intervals = extract_intervals_from_row(date, row)
            if not intervals:
                continue

            secs = summarize_states_for_intervals(date, intervals, machine_name,
                                                  thresholds=thresholds,
                                                  current_threshold=curr_thresh)
            durations_hours = {k: round(v / 3600.0, 2) for k, v in secs.items()}

            img_name = f"{machine_name}_{date}_hinmoku_{idx}.png"
            ok = generate_graph_image_for_intervals(date, intervals,
                                                    os.path.join("static", img_name),
                                                    machine_name)
            image_filename = img_name if ok else None

            intervals_str = " / ".join(
                f"{s.strftime('%H:%M')}-{e.strftime('%H:%M')}" for s, e in intervals)

            items.append({
                "kind": "item", "index": idx,
                "info": {
                    "kikai_no":     row[0],
                    "seiban":       row[1],
                    "tehai_no":     row[2],
                    "hinmoku_no":   row[3],
                    "hinmoku_name": row[4],
                    "status":       (row[8] if len(row) > 8 else ""),
                    "intervals":    intervals_str,
                },
                "durations": durations_hours, "image_filename": image_filename
            })

    year_month = datetime.strptime(date, "%Y-%m-%d").strftime("%Y-%m")
    return render_template("date/overview.html",
                           machine_name=machine_name,
                           date=date, year_month=year_month, items=items)


@app.route("/machine/<machine_name>/date/<date>/table")
def show_table(machine_name, date):
    machine = _get_machine_or_404(machine_name)
    filepath = os.path.join(DATA_DIR, machine_name, f"{date}.csv")
    if not os.path.exists(filepath):
        abort(404)
    rows = []
    with open(filepath, newline='', encoding='utf-8') as f:
        for row in csv.reader(f):
            if len(row) < 5:
                continue
            rows.append({
                "time": row[0], "red": row[1], "yellow": row[2],
                "green": row[3], "current": row[4]
            })
    year_month = datetime.strptime(date, "%Y-%m-%d").strftime("%Y-%m")
    return render_template("date/sensor_data_list.html",
                           machine_name=machine_name,
                           date=date, year_month=year_month, rows=rows)


@app.route("/machine/<machine_name>/date/<date>/status")
def show_status_table(machine_name, date):
    machine = _get_machine_or_404(machine_name)
    thresholds  = machine.get('patlite_thresholds', THRESHOLDS)
    curr_thresh = machine.get('current_threshold', CURRENT_THRESHOLD)
    filepath = os.path.join(DATA_DIR, machine_name, f"{date}.csv")
    if not os.path.exists(filepath):
        abort(404)
    rows = []
    with open(filepath, newline='', encoding='utf-8') as f:
        for row in csv.reader(f):
            if len(row) < 5:
                continue
            red, yellow, green, current = (float(row[1]), float(row[2]),
                                            float(row[3]), float(row[4]))
            lights, machine_action, state, color = get_light_status(
                red, yellow, green, current,
                thresholds=thresholds, current_threshold=curr_thresh)
            rows.append({
                "time": row[0], "red": lights["red"], "yellow": lights["yellow"],
                "green": lights["green"], "machine_action": machine_action,
                "state": state, "color": color
            })
    year_month = datetime.strptime(date, "%Y-%m-%d").strftime("%Y-%m")
    return render_template("date/status_list.html",
                           machine_name=machine_name,
                           date=date, year_month=year_month, rows=rows)


@app.route("/machine/<machine_name>/date/<date>/graph")
def show_graph(machine_name, date):
    machine = _get_machine_or_404(machine_name)
    thresholds  = machine.get('patlite_thresholds', THRESHOLDS)
    curr_thresh = machine.get('current_threshold', CURRENT_THRESHOLD)
    csv_path = os.path.join(DATA_DIR, machine_name, f"{date}.csv")
    if not os.path.exists(csv_path):
        abort(404)

    image_filename = f"{machine_name}_{date}_graph.png"
    image_path     = os.path.join("static", image_filename)
    generate_graph_image_unified(date_str=date, machine_name=machine_name,
                                 thresholds=thresholds, current_threshold=curr_thresh)
    if not os.path.exists(image_path):
        abort(400, description="グラフ画像の生成に失敗しました。")

    year_month = datetime.strptime(date, "%Y-%m-%d").strftime("%Y-%m")
    return render_template("date/graph.html",
                           machine_name=machine_name,
                           date=date, year_month=year_month, image_filename=image_filename)


@app.route("/machine/<machine_name>/date/<date>/summary")
def show_day_summary(machine_name, date):
    machine = _get_machine_or_404(machine_name)
    thresholds  = machine.get('patlite_thresholds', THRESHOLDS)
    curr_thresh = machine.get('current_threshold', CURRENT_THRESHOLD)
    filepath = os.path.join(DATA_DIR, machine_name, f"{date}.csv")
    if not os.path.exists(filepath):
        abort(404)

    states    = ["自動加工中", "手動加工中", "加工完了", "アラーム", "停止"]
    durations = {state: 0 for state in states}

    with open(filepath, newline='', encoding='utf-8') as f:
        for row in csv.reader(f):
            if len(row) < 5:
                continue
            try:
                r, y, g, c = float(row[1]), float(row[2]), float(row[3]), float(row[4])
                _, _, state, _ = get_light_status(r, y, g, c,
                                                  thresholds=thresholds,
                                                  current_threshold=curr_thresh)
                durations[state] += 60
            except Exception:
                continue

    for key in durations:
        durations[key] = round(durations[key] / 3600, 2)

    year_month = datetime.strptime(date, "%Y-%m-%d").strftime("%Y-%m")
    return render_template("date/summary.html",
                           machine_name=machine_name,
                           date=date, year_month=year_month, durations=durations)


@app.route("/machine/<machine_name>/date/<date>/hinmoku")
def show_hinmoku_for_date(machine_name, date):
    machine = _get_machine_or_404(machine_name)
    hinmoku_prefix = machine.get('hinmoku_prefix')
    try:
        datetime.strptime(date, "%Y-%m-%d")
    except ValueError:
        abort(404)

    headers, records, filename = read_hinmoku_csv(date, hinmoku_prefix=hinmoku_prefix)
    if not headers or records is None:
        return render_template(
            "date/hinmoku_list.html",
            machine_name=machine_name,
            has_data=False, date=date,
            message="品目リストはありません"
        )

    display_headers = ["#"] + headers
    indexed_records = [(i, row) for i, row in enumerate(records, start=1)]

    year_month = datetime.strptime(date, "%Y-%m-%d").strftime("%Y-%m")
    return render_template(
        "date/hinmoku_list.html",
        machine_name=machine_name,
        has_data=True, date=date, year_month=year_month,
        headers=display_headers, records=indexed_records, filename=filename
    )


@app.route("/machine/<machine_name>/date/<date>/hinmoku/<int:hinmokuno>")
def show_hinmoku_graph(machine_name, date, hinmokuno):
    machine = _get_machine_or_404(machine_name)
    hinmoku_prefix = machine.get('hinmoku_prefix')
    try:
        datetime.strptime(date, "%Y-%m-%d")
    except ValueError:
        abort(404)

    headers, records, filename = read_hinmoku_csv(date, hinmoku_prefix=hinmoku_prefix)
    if not headers or not records:
        abort(404, description="品目リストがありません。")
    if hinmokuno < 1 or hinmokuno > len(records):
        abort(404, description="指定の品目番号が範囲外です。")

    row       = records[hinmokuno - 1]
    intervals = extract_intervals_from_row(date, row)
    if not intervals:
        abort(400, description="有効な開始/停止区間がありません。")

    image_filename = f"{machine_name}_{date}_hinmoku_{hinmokuno}.png"
    image_path     = os.path.join("static", image_filename)
    ok = generate_graph_image_for_intervals(date, intervals, image_path, machine_name)
    if not ok:
        abort(400, description="グラフ画像の生成に失敗しました。対象区間にデータが無い可能性があります。")

    year_month = datetime.strptime(date, "%Y-%m-%d").strftime("%Y-%m")
    return render_template(
        "hinmoku/graph.html",
        machine_name=machine_name,
        date=date, year_month=year_month, hinmokuno=hinmokuno,
        image_filename=image_filename, row=row, headers=headers
    )


@app.route("/machine/<machine_name>/date/<date>/hinmoku/<int:hinmokuno>/summary")
def show_hinmoku_summary(machine_name, date, hinmokuno):
    machine = _get_machine_or_404(machine_name)
    thresholds     = machine.get('patlite_thresholds', THRESHOLDS)
    curr_thresh    = machine.get('current_threshold', CURRENT_THRESHOLD)
    hinmoku_prefix = machine.get('hinmoku_prefix')
    try:
        datetime.strptime(date, "%Y-%m-%d")
    except ValueError:
        abort(404)

    headers, records, filename = read_hinmoku_csv(date, hinmoku_prefix=hinmoku_prefix)
    if not headers or not records:
        abort(404, description="品目リストがありません。")
    if hinmokuno < 1 or hinmokuno > len(records):
        abort(404, description="指定の品目番号が範囲外です。")

    row       = records[hinmokuno - 1]
    intervals = extract_intervals_from_row(date, row)
    if not intervals:
        abort(404, description="有効な開始/停止区間がありません。")

    secs = summarize_states_for_intervals(date, intervals, machine_name,
                                          thresholds=thresholds,
                                          current_threshold=curr_thresh)
    if secs is None:
        abort(404, description=f"{date}.csv が見つかりません。")

    durations_hours = {k: round(v / 3600.0, 2) for k, v in secs.items()}
    interval_info = {
        "intervals": " / ".join(
            f"{s.strftime('%H:%M')}-{e.strftime('%H:%M')}" for s, e in intervals),
        "clipped_date": date,
        "status": (row[8] if len(row) > 8 else "")
    }

    year_month = datetime.strptime(date, "%Y-%m-%d").strftime("%Y-%m")
    return render_template(
        "hinmoku/summary.html",
        machine_name=machine_name,
        date=date, year_month=year_month, hinmokuno=hinmokuno,
        headers=headers, row=row, durations=durations_hours,
        interval=interval_info, filename=filename
    )


@app.route("/machine/<machine_name>/date/<date>/hinmoku/<int:hinmokuno>/info")
def show_hinmoku_info(machine_name, date, hinmokuno):
    machine = _get_machine_or_404(machine_name)
    thresholds     = machine.get('patlite_thresholds', THRESHOLDS)
    curr_thresh    = machine.get('current_threshold', CURRENT_THRESHOLD)
    hinmoku_prefix = machine.get('hinmoku_prefix')
    try:
        datetime.strptime(date, "%Y-%m-%d")
    except ValueError:
        abort(404)

    headers, records, filename = read_hinmoku_csv(date, hinmoku_prefix=hinmoku_prefix)
    if not headers or not records:
        abort(404, description="品目リストがありません。")
    if hinmokuno < 1 or hinmokuno > len(records):
        abort(404, description="指定の品目番号が範囲外です。")

    row       = records[hinmokuno - 1]
    intervals = extract_intervals_from_row(date, row)
    if not intervals:
        abort(404, description="有効な開始/停止区間がありません。")

    secs = summarize_states_for_intervals(date, intervals, machine_name,
                                          thresholds=thresholds,
                                          current_threshold=curr_thresh)
    if secs is None:
        abort(404, description=f"{date}.csv が見つかりません。")

    durations_hours = {k: round(v / 3600.0, 2) for k, v in secs.items()}
    interval_info = {
        "intervals": " / ".join(
            f"{s.strftime('%H:%M')}-{e.strftime('%H:%M')}" for s, e in intervals),
        "clipped_date": date,
        "status": (row[8] if len(row) > 8 else "")
    }

    year_month = datetime.strptime(date, "%Y-%m-%d").strftime("%Y-%m")
    return render_template(
        "hinmoku/info.html",
        machine_name=machine_name,
        date=date, year_month=year_month, hinmokuno=hinmokuno,
        headers=headers, row=row, durations=durations_hours,
        interval=interval_info, filename=filename
    )


# ===== 後方互換リダイレクト =====

@app.route("/month/<year_month>/overview")
def compat_month_overview(year_month):
    return redirect(url_for('show_month_overview',
                            machine_name=_first_machine_name(),
                            year_month=year_month), 302)


@app.route("/month/<year_month>/graph")
def compat_month_graph(year_month):
    return redirect(url_for('show_month_graph',
                            machine_name=_first_machine_name(),
                            year_month=year_month), 302)


@app.route("/month/<year_month>/summary")
def compat_month_summary(year_month):
    return redirect(url_for('show_month_summary',
                            machine_name=_first_machine_name(),
                            year_month=year_month), 302)


@app.route("/date/<date>/overview")
def compat_date_overview(date):
    return redirect(url_for('show_date_overview',
                            machine_name=_first_machine_name(), date=date), 302)


@app.route("/date/<date>/graph")
def compat_date_graph(date):
    return redirect(url_for('show_graph',
                            machine_name=_first_machine_name(), date=date), 302)


@app.route("/date/<date>/summary")
def compat_date_summary(date):
    return redirect(url_for('show_day_summary',
                            machine_name=_first_machine_name(), date=date), 302)


@app.route("/date/<date>/table")
def compat_date_table(date):
    return redirect(url_for('show_table',
                            machine_name=_first_machine_name(), date=date), 302)


@app.route("/date/<date>/status")
def compat_date_status(date):
    return redirect(url_for('show_status_table',
                            machine_name=_first_machine_name(), date=date), 302)


@app.route("/date/<date>/hinmoku")
def compat_date_hinmoku(date):
    return redirect(url_for('show_hinmoku_for_date',
                            machine_name=_first_machine_name(), date=date), 302)


@app.route("/date/<date>/hinmoku/<int:hinmokuno>")
def compat_hinmoku_graph(date, hinmokuno):
    return redirect(url_for('show_hinmoku_graph',
                            machine_name=_first_machine_name(),
                            date=date, hinmokuno=hinmokuno), 302)


@app.route("/date/<date>/hinmoku/<int:hinmokuno>/summary")
def compat_hinmoku_summary(date, hinmokuno):
    return redirect(url_for('show_hinmoku_summary',
                            machine_name=_first_machine_name(),
                            date=date, hinmokuno=hinmokuno), 302)


@app.route("/date/<date>/hinmoku/<int:hinmokuno>/info")
def compat_hinmoku_info(date, hinmokuno):
    return redirect(url_for('show_hinmoku_info',
                            machine_name=_first_machine_name(),
                            date=date, hinmokuno=hinmokuno), 302)


# ===== OTA ワーカー =====

def _ota_recv(ser, timeout_sec=3.5):
    """OTAレスポンス受信: CRLF終端まで読む（最大200B）"""
    deadline = _time.time() + timeout_sec
    buf = bytearray()
    while _time.time() < deadline:
        if ser.in_waiting:
            buf += ser.read(ser.in_waiting)
            if len(buf) >= 4 and buf[-2] == 0x0D and buf[-1] == 0x0A:
                return bytes(buf)
        _time.sleep(0.005)
    return None


def _ota_worker(job_id, unit_addr, fw_bytes):
    """バックグラウンドスレッドでOTA実行"""
    CHUNK_SIZE  = 128
    total_size  = len(fw_bytes)
    total_crc32 = zlib.crc32(fw_bytes) & 0xFFFFFFFF
    num_chunks  = (total_size + CHUNK_SIZE - 1) // CHUNK_SIZE
    ch          = config['gw_channel']

    def update(progress, status, message=''):
        with g_ota_lock:
            g_ota_jobs[job_id].update({'progress': progress, 'status': status, 'message': message})

    update(0, 'running', 'OTA開始...')

    try:
        with serial_lock:
            ser = g_serial_obj
            if ser is None or not ser.is_open:
                raise RuntimeError('シリアルポートが開いていません。GWを確認してください。')

            # INIT 送信: UI(2B) + total_size(4B,BE) + total_crc32(4B,BE) + chunk_size(2B,BE) + reserved(1B) = 13B
            init_pkt = (b'UI'
                        + struct.pack('>I', total_size)
                        + struct.pack('>I', total_crc32)
                        + struct.pack('>H', CHUNK_SIZE)
                        + b'\x00')
            e220_send_payload(ser, unit_addr, ch, init_pkt)
            resp = _ota_recv(ser, timeout_sec=4.0)
            if resp is None or len(resp) < 4 or resp[2:4] != b'UR':
                raise RuntimeError(f'INIT timeout (resp={resp!r})')
            update(1, 'running', f'INIT OK. {num_chunks}チャンク送信開始...')
            _time.sleep(0.3)

            # DATA 送信ループ
            for seq in range(num_chunks):
                offset = seq * CHUNK_SIZE
                chunk  = fw_bytes[offset:offset + CHUNK_SIZE]
                crc16  = crc16_ccitt(chunk)
                data_pkt = (b'UD'
                            + struct.pack('>HHB', seq, crc16, len(chunk))
                            + chunk)
                e220_send_payload(ser, unit_addr, ch, data_pkt)
                resp = _ota_recv(ser, timeout_sec=3.5)
                if resp is None:
                    raise RuntimeError(f'DATA timeout seq={seq}')
                if len(resp) >= 4 and resp[2:4] == b'UN':
                    err = resp[6] if len(resp) > 6 else 0
                    raise RuntimeError(f'NACK seq={seq} err=0x{err:02X}')
                _time.sleep(0.3)
                progress = int((seq + 1) / num_chunks * 90) + 1
                update(progress, 'running', f'{seq + 1}/{num_chunks} chunks')

            # FIN 送信: UF(2B) + total_size(4B,BE)
            fin_pkt = b'UF' + struct.pack('>I', total_size)
            e220_send_payload(ser, unit_addr, ch, fin_pkt)
            resp = _ota_recv(ser, timeout_sec=10.0)  # CRC32計算に時間がかかるため余裕を持つ
            if resp is None:
                raise RuntimeError('FIN timeout')
            if len(resp) >= 4 and resp[2:4] == b'UF':
                code = resp[4] if len(resp) > 4 else 0
                raise RuntimeError(f'エッジからFAIL応答 code=0x{code:02X}')
            if len(resp) < 4 or resp[2:4] != b'UD':
                raise RuntimeError(f'FIN unexpected response: {resp!r}')
            update(100, 'done', 'OTA完了。エッジが再起動中...')

    except Exception as e:
        update(0, 'failed', str(e))


# ===== メンテナンス =====

@app.route('/maintenance')
def maintenance():
    return render_template('maintenance.html', machines=config['machines'])


@app.route('/api/maint', methods=['POST'])
def api_maint():
    body = request.get_json(force=True, silent=True) or {}
    cmd     = body.get('cmd', '')
    machine = body.get('machine', '')

    if cmd not in ('K', 'V', 'H'):
        return jsonify({'error': 'invalid cmd'}), 400

    # 対象リスト構築
    targets = []
    for m in config['machines']:
        if machine != 'all' and m['name'] != machine:
            continue
        targets.append({'machine': m['name'], 'unit': 'patlite', 'addr': m['patlite_addr']})
        targets.append({'machine': m['name'], 'unit': 'current', 'addr': m['current_addr']})

    if not targets:
        return jsonify({'error': 'no matching machine'}), 404

    req_ids = []
    with g_res_lock:
        for t in targets:
            rid = str(uuid.uuid4())
            g_results[rid] = {'status': 'pending'}
            g_cmd_q.put({'req_id': rid, 'cmd': cmd,
                         'addr': t['addr'], 'machine': t['machine'], 'unit': t['unit']})
            req_ids.append(rid)

    g_maint_event.set()

    return jsonify({'req_ids': req_ids,
                    'targets': [{'machine': t['machine'], 'unit': t['unit']}
                                for t in targets]})


@app.route('/api/maint/result/<req_id>')
def api_maint_result(req_id):
    with g_res_lock:
        entry = g_results.get(req_id)
        if entry is None:
            return jsonify({'status': 'not_found'}), 404
        # 5分以上経過した done 結果を削除（メモリ管理）
        if entry.get('status') == 'done' and _time.time() - entry.get('ts', 0) > 300:
            del g_results[req_id]
            return jsonify({'status': 'not_found'}), 404
        return jsonify(entry)


# ===== OTA Flask ルート =====

@app.route('/maintenance/ota')
def ota_page():
    return render_template('ota.html', machines=config['machines'])


@app.route('/api/ota/upload', methods=['POST'])
def ota_upload():
    """バイナリアップロード → ジョブ登録（実行はまだしない）"""
    f       = request.files.get('firmware')
    machine = request.form.get('machine', '')
    unit    = request.form.get('unit', 'patlite')  # 'patlite' or 'current'

    if not f or not f.filename:
        return jsonify({'error': 'ファームウェアファイルが指定されていません'}), 400
    if not machine:
        return jsonify({'error': '機械名が指定されていません'}), 400

    fw_bytes = f.read()
    if len(fw_bytes) == 0:
        return jsonify({'error': 'ファイルが空です'}), 400
    if len(fw_bytes) > 1 * 1024 * 1024:
        return jsonify({'error': 'ファイルサイズが1MBを超えています'}), 400

    # ターゲットアドレスを解決
    target_machine = None
    for m in config['machines']:
        if m['name'] == machine:
            target_machine = m
            break
    if target_machine is None:
        return jsonify({'error': f'機械 {machine} が見つかりません'}), 404

    unit_addr = target_machine['patlite_addr'] if unit == 'patlite' else target_machine['current_addr']

    job_id = str(uuid.uuid4())
    with g_ota_lock:
        g_ota_jobs[job_id] = {
            'status':    'uploaded',
            'progress':  0,
            'message':   'アップロード完了。OTA開始ボタンを押してください。',
            'machine':   machine,
            'unit':      unit,
            'unit_addr': unit_addr,
            'fw_size':   len(fw_bytes),
            'fw_bytes':  fw_bytes,
            'ts':        _time.time(),
        }
    return jsonify({'job_id': job_id, 'fw_size': len(fw_bytes)})


@app.route('/api/ota/start/<job_id>', methods=['POST'])
def ota_start(job_id):
    """アップロード済みジョブを実行開始"""
    with g_ota_lock:
        job = g_ota_jobs.get(job_id)
    if job is None:
        return jsonify({'error': 'ジョブが見つかりません'}), 404
    if job['status'] not in ('uploaded', 'failed'):
        return jsonify({'error': f'ジョブは既に {job["status"]} 状態です'}), 400

    # 実行中ジョブが他にないか確認
    with g_ota_lock:
        for jid, j in g_ota_jobs.items():
            if jid != job_id and j.get('status') == 'running':
                return jsonify({'error': '別のOTAジョブが実行中です'}), 409
        g_ota_jobs[job_id]['status'] = 'running'

    fw_bytes  = job['fw_bytes']
    unit_addr = job['unit_addr']
    threading.Thread(
        target=_ota_worker, args=(job_id, unit_addr, fw_bytes), daemon=True
    ).start()
    return jsonify({'status': 'started'})


@app.route('/api/ota/progress/<job_id>')
def ota_progress(job_id):
    with g_ota_lock:
        job = g_ota_jobs.get(job_id)
    if job is None:
        return jsonify({'status': 'not_found'}), 404
    # fw_bytes はレスポンスに含めない
    safe = {k: v for k, v in job.items() if k != 'fw_bytes'}
    return jsonify(safe)


if __name__ == "__main__":
    # use_reloader=False: werkzeug の2重プロセス起動を防ぎ polling_loop が1本だけ動く
    app.run(debug=True, host="0.0.0.0", port=5000, use_reloader=False)
