#!/usr/bin/env python3
import re
import csv
import os
import signal
from datetime import datetime
from pymavlink import mavutil

running = True
message_buffer = ""
expecting_quat_line = False

def signal_handler(sig, frame):
    global running
    running = False
    print("\n終了信号受信, 停止します…")

signal.signal(signal.SIGINT, signal_handler)

def create_csv_filepath():
    now = datetime.now().strftime("%Y%m%d_%H%M%S")
    directory = os.path.expanduser("~/LOGS_Pixhawk6c")
    os.makedirs(directory, exist_ok=True)
    return os.path.join(directory, f"{now}_EFandQ.csv")

def process_statustext_line(text):
    """
    2行に分割された EF=... と Q=... を組み立てる
    """
    global message_buffer, expecting_quat_line

    text = text.strip()
    if text.startswith("EF="):
        message_buffer = text
        expecting_quat_line = True
        return None
    if expecting_quat_line and text.startswith("Q="):
        complete = message_buffer + " " + text
        message_buffer = ""
        expecting_quat_line = False
        return complete
    message_buffer = ""
    expecting_quat_line = False
    return None

def parse_ef_q(msg):
    """
    完全な EF=..., Q=... 文字列を解析して辞書で返す
    """
    m = re.match(
        r"EF=([\-\d\.]+),([\-\d\.]+),([\-\d\.]+) "
        r"Q=([\d\.\-]+),([\d\.\-]+),([\d\.\-]+),([\d\.\-]+)",
        msg
    )
    if not m:
        return None
    fx, fy, fz, q1, q2, q3, q4 = map(float, m.groups())
    magnitude = (fx*fx + fy*fy + fz*fz)**0.5
    return {
        "Force_X": fx,
        "Force_Y": fy,
        "Force_Z": fz,
        "Force_Mag": magnitude,
        "Q1": q1,
        "Q2": q2,
        "Q3": q3,
        "Q4": q4
    }

def main():
    # CSV準備
    csv_path = create_csv_filepath()
    with open(csv_path, "w", newline="", encoding="utf-8") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([
            "Timestamp",
            "Force_X_N", "Force_Y_N", "Force_Z_N", "Force_Magnitude_N",
            "Quat_Q1", "Quat_Q2", "Quat_Q3", "Quat_Q4"
        ])
        print(f"CSV 保存先: {csv_path}")

        # MAVLink接続
        master = mavutil.mavlink_connection("/dev/ttyAMA0", baud=1000000, rtscts=True)
        master.wait_heartbeat(timeout=5)
        print("Heartbeat 受信: 接続完了")

        # データストリーム要求 (10Hz)
        master.mav.request_data_stream_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
            10, 1
        )
        print("監視開始… Ctrl+C で終了")

        record_count = 0
        while running:
            msg = master.recv_match(type="STATUSTEXT", blocking=True, timeout=1)
            if not msg:
                continue
            # msg.text は既に str
            text = msg.text.strip()
            complete = process_statustext_line(text)
            if not complete:
                continue
            data = parse_ef_q(complete)
            if not data:
                continue

            # タイムスタンプ
            ts = datetime.now().strftime("%Y/%m/%d %H:%M:%S.%f")[:-3]
            row = [
                ts,
                data["Force_X"], data["Force_Y"], data["Force_Z"], data["Force_Mag"],
                data["Q1"], data["Q2"], data["Q3"], data["Q4"]
            ]
            writer.writerow(row)
            csvfile.flush()
            record_count += 1

            if record_count % 5 == 0:
                print(f"{ts} : 記録 #{record_count} "
                      f"EF=({data['Force_X']:.3f},{data['Force_Y']:.3f},{data['Force_Z']:.3f}) "
                      f"Q=({data['Q1']:.4f},{data['Q2']:.4f},{data['Q3']:.4f},{data['Q4']:.4f})")

        # 終了処理
        print("\n停止中…")
        master.close()
        print(f"総記録数: {record_count}")
        print("CSV ファイルを保存しました。")

if __name__ == "__main__":
    main()
