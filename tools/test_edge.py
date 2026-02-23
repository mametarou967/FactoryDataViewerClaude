#!/usr/bin/env python3
"""
test_edge.py  –  Phase 2 Step1 エッジユニット テストスクリプト

Ubuntu PC + USB E220（GW役）からコマンドを送信し、応答を確認する。

使い方:
    python3 test_edge.py                          # デフォルト設定で実行
    python3 test_edge.py --port /dev/ttyUSB1      # ポート指定
    python3 test_edge.py --addr 0x0201 --ch 3     # アドレス・チャンネル指定

事前準備:
    pip install pyserial
    sudo usermod -a -G dialout $USER  # 初回のみ（要ログアウト→ログイン）
"""

import serial
import time
import argparse
import sys

# ===== デフォルト設定 =====
DEFAULT_PORT    = "/dev/ttyUSB0"
DEFAULT_BAUD    = 9600
DEFAULT_ADDR    = 0x0101  # エッジのLoRaアドレス（0xMMTT形式）
DEFAULT_CHANNEL = 2       # LoRaチャンネル (JP版 CH2=921.0MHz, Zone A)

TIMEOUT_MS  = 2500   # 応答タイムアウト [ms]  ※9375bps(SF6/BW125k)でRT~1100ms+100ms待機→2500msで22台×全タイムアウト=55秒以内
RETRY_COUNT = 0     # 再送なし（再送するとLoRa TX同士が衝突する）

# ===== エラーコード =====
ERROR_CODES = {
    0x01: "センサー異常",
    0x02: "未対応コマンド",
    0x03: "処理タイムアウト",
}


def hex_dump(label, data):
    body = ' '.join(f'{b:02X}' for b in data)
    print(f"  {label} ({len(data)} bytes): {body}")


def send_command(ser, edge_addr, channel, cmd_char, timeout_ms=TIMEOUT_MS, retries=RETRY_COUNT):
    """
    コマンドを送信し、レスポンスのバイト列を返す（タイムアウト時はNone）。

    E220固定アドレスモードの実書き込み:
        [DEST_ADDR_H][DEST_ADDR_L][CH][CMD][CR][LF]
    """
    addr_h = (edge_addr >> 8) & 0xFF
    addr_l = edge_addr & 0xFF
    packet = bytes([addr_h, addr_l, channel, ord(cmd_char), 0x0D, 0x0A])

    for attempt in range(retries + 1):
        if attempt > 0:
            print(f"  再送 ({attempt}/{retries})...")

        ser.reset_input_buffer()
        ser.write(packet)
        hex_dump("TX", packet)

        # CRLFで終端されるまで読み取る
        deadline = time.time() + timeout_ms / 1000.0
        buf = bytearray()
        while time.time() < deadline:
            if ser.in_waiting:
                buf += ser.read(ser.in_waiting)
                if len(buf) >= 3 and buf[-2] == 0x0D and buf[-1] == 0x0A:
                    hex_dump("RX", buf)
                    return bytes(buf)
            time.sleep(0.005)

        if buf:
            hex_dump("RX(partial)", buf)
        else:
            print(f"  RX: 0 bytes (timeout)")

    return None  # タイムアウト


def parse_response(resp, cmd_char):
    """
    レスポンスバイト列をパースして辞書で返す。

    共通フォーマット（GW側が受信するペイロード）:
        [ADDR_H][ADDR_L][CMD][data...][CR][LF]
    """
    if resp is None:
        return None

    if len(resp) < 5:
        return {"error": f"レスポンスが短すぎる ({len(resp)} bytes)"}
    if resp[-2] != 0x0D or resp[-1] != 0x0A:
        return {"error": "CRLFなし"}

    result = {
        "raw":       resp.hex(' '),
        "from_addr": f"0x{resp[0]:02X}{resp[1]:02X}",
        "cmd":       chr(resp[2]),
    }

    # エラー応答
    if resp[2] == ord('E'):
        code = resp[3] if len(resp) >= 6 else 0
        result["error_code"] = f"0x{code:02X} ({ERROR_CODES.get(code, '不明')})"
        return result

    # コマンド不一致
    if resp[2] != ord(cmd_char):
        result["error"] = f"コマンド不一致 (期待:{cmd_char} 実際:{chr(resp[2])})"
        return result

    data = resp[3:-2]  # CRLF を除いたデータ部

    if cmd_char == 'K':
        pass  # データなし（アドレス + K だけ）

    elif cmd_char == 'V':
        if len(data) >= 3:
            result["version"] = f"{data[0]}.{data[1]}.{data[2]}"

    elif cmd_char == 'H':
        if len(data) >= 6:
            dip = data[0]
            result["dip"]          = f"0x{dip:02X}"
            result["dip_patlite"]  = bool(dip & 0x01)
            result["dip_current"]  = bool(dip & 0x02)
            result["e220_addr"]    = f"0x{data[1]:02X}{data[2]:02X}"
            result["e220_ch"]      = data[3]
            result["air_rate"]     = data[4]
            result["tx_power"]     = data[5]

    return result


def run_test(port, baud, edge_addr, channel):
    print(f"{'='*52}")
    print(f"  Edge Unit テスト  –  Phase 2 Step1")
    print(f"    ポート  : {port}  ({baud} bps)")
    print(f"    エッジ  : 0x{edge_addr:04X}  CH={channel}")
    print(f"{'='*52}\n")

    try:
        ser = serial.Serial(port, baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"[ERROR] ポートを開けません: {e}")
        sys.exit(1)

    time.sleep(0.2)

    results = {}

    for cmd in ['K', 'V', 'H']:
        print(f"[{cmd}] ", end="", flush=True)
        resp   = send_command(ser, edge_addr, channel, cmd)
        parsed = parse_response(resp, cmd)

        if parsed is None:
            print("TIMEOUT")
            results[cmd] = False

        elif "error" in parsed:
            print(f"ERROR: {parsed['error']}")
            print(f"     raw: {resp.hex(' ') if resp else 'None'}")
            results[cmd] = False

        elif "error_code" in parsed:
            print(f"エラー応答: {parsed['error_code']}")
            print(f"     raw: {parsed['raw']}")
            results[cmd] = False

        else:
            results[cmd] = True
            if cmd == 'K':
                print(f"OK  from={parsed['from_addr']}")
            elif cmd == 'V':
                print(f"OK  version={parsed.get('version', '?')}  from={parsed['from_addr']}")
            elif cmd == 'H':
                print(f"OK  from={parsed['from_addr']}")
                print(f"     DIP       : {parsed['dip']} "
                      f"(patlite={parsed['dip_patlite']}, current={parsed['dip_current']})")
                print(f"     E220 addr : {parsed['e220_addr']}")
                print(f"     E220 ch   : {parsed['e220_ch']}")
                print(f"     air_rate  : {parsed['air_rate']}")
                print(f"     tx_power  : {parsed['tx_power']}")
            print(f"     raw: {parsed['raw']}")

        time.sleep(0.3)

    ser.close()

    print(f"\n{'='*52}")
    passed = sum(results.values())
    total  = len(results)
    print(f"  結果: {passed}/{total} passed  ", end="")
    if passed == total:
        print("ALL PASS")
    else:
        failed = [k for k, v in results.items() if not v]
        print(f"FAILED: {failed}")
    print(f"{'='*52}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Edge unit test via E220 LoRa")
    parser.add_argument("--port", default=DEFAULT_PORT,        help="シリアルポート (default: /dev/ttyUSB0)")
    parser.add_argument("--addr", default=hex(DEFAULT_ADDR),   help="エッジアドレス (default: 0x0101)")
    parser.add_argument("--ch",   default=DEFAULT_CHANNEL, type=int, help="LoRaチャンネル (default: 18)")
    parser.add_argument("--baud", default=DEFAULT_BAUD,    type=int, help="ボーレート (default: 115200)")
    args = parser.parse_args()

    run_test(
        port      = args.port,
        baud      = args.baud,
        edge_addr = int(args.addr, 16),
        channel   = args.ch,
    )
