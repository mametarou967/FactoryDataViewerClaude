#!/usr/bin/env python3
"""
gw_e220_config.py  –  GW側 E220 設定ツール

E220-900T22S レジスタマップ（6バイト）:
    0x00: ADDH  0x01: ADDL  0x02: REG0(SPED)
    0x03: REG1  0x04: CHAN   0x05: REG3(OPTION)
    ※ NETIDフィールドは存在しない

事前準備:
    GW E220モジュールのDIPスイッチを M0=HIGH, M1=HIGH（コンフィグモード）にしてから実行。
    実行後は M0=LOW, M1=LOW（通常モード）に戻すこと。

使い方:
    python3 tools/gw_e220_config.py               # 現在設定を読み取って表示
    python3 tools/gw_e220_config.py --write       # GW設定を書き込む
"""

import serial
import time
import argparse
import sys

DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 9600   # E220コンフィグモードは常に9600bps固定

# ===== GWに書き込む設定値 =====
GW_ADDH    = 0x00   # GWアドレス上位（0x0000 固定）
GW_ADDL    = 0x00   # GWアドレス下位
GW_REG0    = 0x64   # 9600bps UART / 9375bps air (SF6/BW125kHz)
                    #   bits[7:5]=011(9600bps) bits[4:0]=00100(SF6/BW125k)
GW_REG1    = 0x01   # 200バイトパケット / RSSI無効 / 13dBm(22S JPデフォルト)
                    #   bits[7:6]=00(200B) bit5=0(RSSI無効) bits[3:0]=0001(13dBm)
GW_CHANNEL = 2      # JP版: BW125kHz→921.0MHz, BW500kHz→921.2MHz (Zone A: 920.6-923.4 MHz)
GW_REG3    = 0x40   # 通常送信モード（固定アドレス）bit6=1

# 表示用マップ (JP版: E220-900T22S/22L(JP) firmware v2.0)
# REG0 bits[4:0] = エアレート(SF/BW複合5ビット): bits[1:0]=BW, bits[4:2]=SF-5
AIR_RATE_MAP = {
    # BW125kHz (CH200kHz間隔, Zone A: CH0-14)
    0x00: "15625bps(SF5/BW125k)", 0x04: "9375bps(SF6/BW125k)",
    0x08: "5469bps(SF7/BW125k)",  0x0C: "3125bps(SF8/BW125k)",
    0x10: "1758bps(SF9/BW125k)",
    # BW250kHz
    0x01: "31250bps(SF5/BW250k)", 0x05: "18750bps(SF6/BW250k)",
    0x09: "10938bps(SF7/BW250k)", 0x0D: "6250bps(SF8/BW250k)",
    0x11: "3516bps(SF9/BW250k)",  0x15: "1953bps(SF10/BW250k)",
    # BW500kHz
    0x02: "62500bps(SF5/BW500k)", 0x06: "37500bps(SF6/BW500k)",
    0x0A: "21875bps(SF7/BW500k)", 0x0E: "12500bps(SF8/BW500k)",
    0x12: "7031bps(SF9/BW500k)",  0x16: "3906bps(SF10/BW500k)",
    0x1A: "2148bps(SF11/BW500k)",
}
BAUD_MAP     = {0: "1200", 1: "2400", 2: "4800", 3: "9600",
                4: "19200", 5: "38400", 6: "57600", 7: "115200"}
# REG1 bits[3:0] = TX電力 (JP版 22S: 0=未定義, 1=13dBm(default), 2=7dBm, 3=0dBm, 4-15=1-12dBm)
POWER_MAP    = {
    0x0: "N/A(未定義)", 0x1: "13dBm", 0x2: "7dBm",  0x3: "0dBm",
    0x4: "1dBm",  0x5: "2dBm",  0x6: "3dBm",  0x7: "4dBm",
    0x8: "5dBm",  0x9: "6dBm",  0xA: "7dBm",  0xB: "8dBm",
    0xC: "9dBm",  0xD: "10dBm", 0xE: "11dBm", 0xF: "12dBm",
}


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
    """E220設定を読み取る（コンフィグモードで実行すること）
    コマンド: C1 00 06 → レジスタ0x00〜0x05を6バイト読み取り
    レスポンス: [C1][00][06][ADDH][ADDL][REG0][REG1][CHAN][REG3] (9バイト)
    """
    ser.reset_input_buffer()

    cmd = bytes([0xC1, 0x00, 0x06])
    ser.write(cmd)
    time.sleep(0.5)
    resp = ser.read(64)

    print(f"  RAW応答: {resp.hex(' ')} ({len(resp)} bytes)")

    if len(resp) < 9:
        print(f"  [ERROR] 応答が短すぎます（期待: 9バイト）")
        return None
    if resp[0] != 0xC1:
        print(f"  [ERROR] 先頭バイトが不正 (0x{resp[0]:02X}, 期待: 0xC1)")
        return None

    data = resp[3:9]  # ヘッダ3バイト後の6バイト
    return {
        "addH":    data[0],
        "addL":    data[1],
        "reg0":    data[2],
        "reg1":    data[3],
        "channel": data[4],
        "reg3":    data[5],
    }


def print_config(cfg, label="設定"):
    # JP版: REG0 bits[7:5]=UARTボーレート, bits[4:0]=エアレート(SF/BW複合)  ※パリティフィールドなし
    # JP版: REG1 bits[3:0]=TX電力(22S最大13dBm)
    baud_bits = (cfg["reg0"] >> 5) & 0x07
    air_bits  =  cfg["reg0"] & 0x1F           # JP版: bits[4:0] (5ビット)
    pwr_bits  =  cfg["reg1"] & 0x0F           # JP版 22S: bits[3:0] (4ビット)
    fixed     = bool(cfg["reg3"] & 0x40)      # bit6=1: 通常送信(固定アドレス)

    # BW依存の周波数計算
    bw_idx   = air_bits & 0x03
    bw_base  = [920.6, 920.7, 920.8]
    freq     = bw_base[bw_idx] + cfg["channel"] * 0.2 if bw_idx < 3 else 0.0
    bw_label = ["BW125k", "BW250k", "BW500k", "??"][bw_idx if bw_idx < 3 else 3]

    print(f"  [{label}]")
    print(f"    アドレス          : 0x{cfg['addH']:02X}{cfg['addL']:02X}")
    print(f"    UARTボーレート    : {BAUD_MAP.get(baud_bits, '?')} bps")
    print(f"    エアレート        : {AIR_RATE_MAP.get(air_bits, f'不明(0x{air_bits:02X})')}")
    print(f"    チャンネル        : {cfg['channel']} ({freq:.1f} MHz, {bw_label})")
    print(f"    TX電力            : {POWER_MAP.get(pwr_bits, '?')}")
    print(f"    送信モード        : {'通常(固定アドレス) ✓' if fixed else '透過モード'}")
    print(f"    REG0=0x{cfg['reg0']:02X} REG1=0x{cfg['reg1']:02X} REG3=0x{cfg['reg3']:02X}")


def write_config(ser):
    """GW設定を書き込む（コンフィグモードで実行すること）
    コマンド: C0 00 06 + 6バイト → E2PROMに永続書き込み
    レスポンス: [C1][00][06][書き込んだデータ] (9バイト)
    """
    ser.reset_input_buffer()

    cmd = bytes([
        0xC0, 0x00, 0x06,
        GW_ADDH, GW_ADDL,
        GW_REG0, GW_REG1, GW_CHANNEL, GW_REG3
    ])
    print(f"  書き込みコマンド: {cmd.hex(' ')}")
    ser.write(cmd)
    time.sleep(0.5)
    resp = ser.read(64)
    print(f"  RAW応答: {resp.hex(' ')} ({len(resp)} bytes)")

    if len(resp) < 9 or resp[0] != 0xC1:
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
        ser.close()
        return

    if not do_write:
        ser.close()
        return

    # ---- 書き込む設定を表示 ----
    new_cfg = {
        "addH": GW_ADDH, "addL": GW_ADDL,
        "reg0": GW_REG0, "reg1": GW_REG1,
        "channel": GW_CHANNEL, "reg3": GW_REG3
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
