#!/usr/bin/env python3
"""
gw_e220_config.py  –  GW側 E220 設定ツール

事前準備:
    GW E220モジュールのDIPスイッチを M0=HIGH, M1=HIGH（コンフィグモード）にしてから実行。
    実行後は M0=LOW, M1=LOW（通常モード）に戻すこと。

使い方:
    python3 tools/gw_e220_config.py               # 現在設定を読み取って表示
    python3 tools/gw_e220_config.py --write       # GW設定を書き込む
    python3 tools/gw_e220_config.py --port /dev/ttyUSB1 --write
"""

import serial
import time
import argparse
import sys

DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 9600   # E220のデフォルトボーレート（コンフィグモード時）

# ===== GWに書き込む設定値 =====
GW_ADDH    = 0x00   # GWアドレス上位（0x0000 固定）
GW_ADDL    = 0x00   # GWアドレス下位
GW_NETID   = 0x00   # ネットワークID
GW_REG0    = 0xE4   # 115200bps / 8N1 / 9.6kbps air rate
                    #   bits[7:5]=111(115200) bits[4:3]=00(8N1) bits[2:0]=100(9.6k)
GW_REG1    = 0x00   # 200バイトパケット / RSSI無効 / 22dBm
GW_CHANNEL = 18     # ch=18（工場1デフォルト）
GW_REG3    = 0x40   # 固定アドレスモード
                    #   bit6=1(fixed addressing) / それ以外0

# REG0のエアレート対応表
AIR_RATE_MAP = {0: "0.3k", 1: "1.2k", 2: "2.4k", 3: "4.8k",
                4: "9.6k", 5: "19.2k", 6: "38.4k", 7: "62.5k"}
BAUD_MAP     = {0: "1200", 1: "2400", 2: "4800", 3: "9600",
                4: "19200", 5: "38400", 6: "57600", 7: "115200"}
POWER_MAP    = {0: "22dBm", 1: "17dBm", 2: "13dBm", 3: "10dBm"}


def open_port(port, baud):
    try:
        s = serial.Serial(port, baud, timeout=1)
        time.sleep(0.3)
        s.reset_input_buffer()
        return s
    except serial.SerialException as e:
        print(f"[ERROR] ポートを開けません: {e}")
        sys.exit(1)


def read_config(ser):
    """E220設定を読み取る（コンフィグモードで実行すること）"""
    ser.reset_input_buffer()

    # C1 00 07: レジスタ0x00〜0x06を7バイト読み取り
    cmd = bytes([0xC1, 0x00, 0x07])
    ser.write(cmd)

    time.sleep(0.5)
    resp = ser.read(64)  # 最大64バイト読み取り

    print(f"  RAW応答: {resp.hex(' ')} ({len(resp)} bytes)")

    if len(resp) < 10:
        print(f"  [ERROR] 応答が短すぎます（期待: 10バイト以上）")
        return None

    if resp[0] != 0xC1:
        print(f"  [ERROR] 先頭バイトが不正 (0x{resp[0]:02X}, 期待: 0xC1)")
        return None

    # ヘッダ(3バイト) + データ(7バイト)
    data = resp[3:10]
    cfg = {
        "addH":    data[0],
        "addL":    data[1],
        "netId":   data[2],
        "reg0":    data[3],
        "reg1":    data[4],
        "channel": data[5],
        "reg3":    data[6],
    }
    return cfg


def print_config(cfg, label="設定"):
    baud_bits = (cfg["reg0"] >> 5) & 0x07
    air_bits  = cfg["reg0"] & 0x07
    pwr_bits  = cfg["reg1"] & 0x03
    fixed     = bool(cfg["reg3"] & 0x40)

    print(f"  [{label}]")
    print(f"    アドレス    : 0x{cfg['addH']:02X}{cfg['addL']:02X}")
    print(f"    ネットID    : 0x{cfg['netId']:02X}")
    print(f"    UARTボーレート: {BAUD_MAP.get(baud_bits, '?')} bps")
    print(f"    エアレート  : {AIR_RATE_MAP.get(air_bits, '?')} bps")
    print(f"    チャンネル  : {cfg['channel']} (freq={850.125 + cfg['channel']:.3f} MHz)")
    print(f"    TX電力      : {POWER_MAP.get(pwr_bits, '?')}")
    print(f"    固定アドレスモード: {'有効' if fixed else '無効（透過モード）'}")
    print(f"    REG0=0x{cfg['reg0']:02X} REG1=0x{cfg['reg1']:02X} REG3=0x{cfg['reg3']:02X}")


def write_config(ser):
    """GW設定を書き込む（コンフィグモードで実行すること）"""
    ser.reset_input_buffer()

    # C0: E2PROMに永続書き込み
    cmd = bytes([
        0xC0, 0x00, 0x07,
        GW_ADDH, GW_ADDL, GW_NETID,
        GW_REG0, GW_REG1, GW_CHANNEL,
        GW_REG3
    ])
    print(f"  書き込みコマンド: {cmd.hex(' ')}")
    ser.write(cmd)

    time.sleep(0.5)
    resp = ser.read(64)
    print(f"  RAW応答: {resp.hex(' ')} ({len(resp)} bytes)")

    if len(resp) < 10 or resp[0] != 0xC1:
        print("  [ERROR] 書き込み応答が不正")
        return False

    print("  [OK] 書き込み成功")
    return True


def run(port, baud, do_write):
    print(f"{'='*52}")
    print(f"  GW E220 設定ツール")
    print(f"    ポート: {port} ({baud} bps)")
    print(f"    ※ M0=HIGH, M1=HIGH（コンフィグモード）で実行すること")
    print(f"{'='*52}\n")

    ser = open_port(port, baud)

    # ---- 現在設定を読み取り ----
    print("[1] 現在の設定を読み取り中...")
    cfg = read_config(ser)
    if cfg:
        print_config(cfg, "現在")
    else:
        print("  読み取り失敗（M0/M1がHIGHになっているか確認してください）")

    if not do_write:
        ser.close()
        return

    # ---- 書き込む設定を表示 ----
    new_cfg = {
        "addH": GW_ADDH, "addL": GW_ADDL, "netId": GW_NETID,
        "reg0": GW_REG0, "reg1": GW_REG1, "channel": GW_CHANNEL, "reg3": GW_REG3
    }
    print("\n[2] 書き込む設定:")
    print_config(new_cfg, "書き込み予定")

    ans = input("\nこの設定を書き込みますか？ [y/N]: ").strip().lower()
    if ans != "y":
        print("キャンセルしました")
        ser.close()
        return

    # ---- 書き込み ----
    print("\n[3] 書き込み中...")
    if write_config(ser):
        print("\n[4] 書き込み後の設定を確認中...")
        time.sleep(0.2)
        cfg_after = read_config(ser)
        if cfg_after:
            print_config(cfg_after, "書き込み後")

    ser.close()
    print(f"\n{'='*52}")
    print("  完了。DIPスイッチを M0=LOW, M1=LOW に戻してください。")
    print(f"{'='*52}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="GW E220 設定ツール")
    parser.add_argument("--port",  default=DEFAULT_PORT, help="シリアルポート")
    parser.add_argument("--baud",  default=DEFAULT_BAUD, type=int, help="ボーレート（デフォルト: 9600）")
    parser.add_argument("--write", action="store_true",  help="設定を書き込む")
    args = parser.parse_args()

    run(args.port, args.baud, args.write)
