#!/usr/bin/env python3
#  最新の Pixhawk6C *.BIN* から GPS 生データだけを抽出し
#  日本時間タイムスタンプ付き CSV として ~/LOGS1 に保存するワンファイル
#    • UART : /dev/ttyAMA0 1 000 000 bps (RTS/CTS 有効)
#    • SD : /media/px4_sd/APM/LOGS/nnnnnnnn.BIN
#    • 出力 : ~/LOGS1/20250629_130215_gps.csv

import glob, os, time, csv, datetime, pytz
from pathlib import Path
from pymavlink import mavutil          # sudo pip3 install pymavlink

# ────────────────────────────── ユーザー設定 ─────────────────────────────
LOG_DIR  = "/media/px4_sd/APM/LOGS"    # FC が自動生成するログフォルダ
SERIAL   = "/dev/ttyAMA0"              # RasPi ↔ Pixhawk6C の UART
BAUD     = 1_000_000                   # DSHOT 対応 FC の標準ボーレート
CSV_DIR  = Path.home() / "LOGS1"       # 保存先 ~/LOGS1
CSV_DIR.mkdir(exist_ok=True)
# ────────────────────────────────────────────────────────────────────────


def latest_bin(path: str) -> str | None:
    """LOG_DIR で更新が最も新しい *.BIN* を返す"""
    bins = glob.glob(os.path.join(path, "*.BIN"))
    return max(bins, key=os.path.getmtime) if bins else None


def wait_until_closed(fpath: str, interval=1.0, repeat=3) -> None:
    """サイズが repeat 回連続で変化しなくなるまで待機 = 書き込み完了"""
    last, stable = -1, 0
    while stable < repeat:
        now = os.path.getsize(fpath)
        stable = stable + 1 if now == last else 0
        last = now
        time.sleep(interval)


def extract_gps(bin_file: str, csv_path: Path) -> None:
    """BIN 内の GPS_RAW_INT / GLOBAL_POSITION_INT を CSV 出力"""
    mav = mavutil.mavlink_connection(bin_file)
    with csv_path.open("w", newline="") as f:
        w = csv.writer(f); w.writerow(["time_us", "lat_deg", "lon_deg", "alt_m"])
        while True:
            msg = mav.recv_match(
                type=["GPS_RAW_INT", "GLOBAL_POSITION_INT"],
                blocking=False)
            if msg is None:
                break
            if msg.get_type() == "GPS_RAW_INT":
                t, lat, lon, alt = (
                    msg.time_usec,
                    msg.lat / 1e7,
                    msg.lon / 1e7,
                    msg.alt / 1000)
            else:  # GLOBAL_POSITION_INT
                t, lat, lon, alt = (
                    msg.time_boot_ms * 1000,
                    msg.lat / 1e7,
                    msg.lon / 1e7,
                    msg.alt / 1000)
            w.writerow([t, lat, lon, alt])
    print("✓ GPS extracted →", csv_path)


def main() -> None:
    # ① FC と通信確認（RTS/CTS）
    master = mavutil.mavlink_connection(SERIAL, baud=BAUD, rtscts=True)
    try:
        master.wait_heartbeat(timeout=5)
        print("✓ Pixhawk6C heartbeat OK")
    except mavutil.mavutil.mavlink.MAVError:
        print("⚠ Heartbeat not received（配線 / BAUD 要確認）")

    # ② 最新の BIN を取得
    bin_file = latest_bin(LOG_DIR)
    if bin_file is None:
        print("⚠ BIN が見つかりません:", LOG_DIR); return
    print("最新ログ =", bin_file)

    # ③ 書き込み完了を待ってから解析
    wait_until_closed(bin_file)

    # ④ 日本時間タイムスタンプ付き CSV 名
    jst   = pytz.timezone("Asia/Tokyo")
    stamp = datetime.datetime.now(jst).strftime("%Y%m%d_%H%M%S")
    csv_path = CSV_DIR / f"{stamp}_gps.csv"

    # ⑤ 抽出実行
    extract_gps(bin_file, csv_path)


if __name__ == "__main__":
    main()
