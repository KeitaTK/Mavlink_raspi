#!/usr/bin/env python3
# encoding: utf-8

import re
import csv
import os
import signal
import math
from datetime import datetime
from pymavlink import mavutil

running = True
message_buffer = ""
expecting = None  # None, "Q", "Gain"

def signal_handler(sig, frame):
    global running
    running = False
    print("\n終了信号受信, 停止します…")

signal.signal(signal.SIGINT, signal_handler)

def create_csv_filepath():
    now = datetime.now().strftime("%Y%m%d_%H%M%S")
    directory = os.path.expanduser("~/LOGS_Pixhawk6c")
    os.makedirs(directory, exist_ok=True)
    return os.path.join(directory, f"{now}_EFQGain.csv")

def process_statustext_line(text):
    global message_buffer, expecting
    text = text.strip()
    if text.startswith("EF="):
        message_buffer = text
        expecting = "Q"
        return None
    if expecting == "Q" and text.startswith("Q="):
        message_buffer += " " + text
        expecting = "Gain"
        return None
    if expecting == "Gain" and text.startswith("Gain="):
        complete = message_buffer + " " + text
        message_buffer = ""
        expecting = None
        return complete
    message_buffer = ""
    expecting = None
    return None

def parse_ef_q_gain(msg):
    m = re.match(
        r"EF=([-\d\.]+),([-\d\.]+),([-\d\.]+) "
        r"Q=([-\d\.]+),([-\d\.]+),([-\d\.]+),([-\d\.]+) "
        r"Gain=([-\d\.]+)",
        msg
    )
    if not m:
        return None
    fx, fy, fz, q1, q2, q3, q4, gain = map(float, m.groups())
    magnitude = math.sqrt(fx*fx + fy*fy + fz*fz)
    return {
        "Force_X": fx,
        "Force_Y": fy,
        "Force_Z": fz,
        "Force_Mag": magnitude,
        "Q1": q1,
        "Q2": q2,
        "Q3": q3,
        "Q4": q4,
        "Gain": gain
    }

def main():
    csv_path = create_csv_filepath()
    with open(csv_path, "w", newline="", encoding="utf-8") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([
            "Timestamp",
            "Force_X_N", "Force_Y_N", "Force_Z_N", "Force_Magnitude_N",
            "Quat_Q1", "Quat_Q2", "Quat_Q3", "Quat_Q4",
            "Correction_Gain"
        ])
        print(f"CSV 保存先: {csv_path}")

        master = mavutil.mavlink_connection("/dev/ttyAMA0", baud=1000000, rtscts=True)
        master.wait_heartbeat(timeout=5)
        print("Heartbeat 受信: 接続完了")

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
            complete = process_statustext_line(msg.text)
            if not complete:
                continue
            data = parse_ef_q_gain(complete)
            if not data:
                continue

            # CSV 書き込み
            ts = datetime.now().strftime("%Y/%m/%d %H:%M:%S.%f")[:-3]
            row = [
                ts,
                data["Force_X"], data["Force_Y"], data["Force_Z"], data["Force_Mag"],
                data["Q1"], data["Q2"], data["Q3"], data["Q4"],
                data["Gain"]
            ]
            writer.writerow(row)
            csvfile.flush()
            record_count += 1

            # コンソール表示: Roll, Pitch に変換して表示
            if record_count % 5 == 0:
                q0, q1, q2, q3 = data["Q1"], data["Q2"], data["Q3"], data["Q4"]
                # Roll = atan2(2*(q0*q1+q2*q3), 1-2*(q1^2+q2^2))
                roll = math.degrees(math.atan2(
                    2*(q0*q1 + q2*q3),
                    1 - 2*(q1*q1 + q2*q2)
                ))
                # Pitch = asin(2*(q0*q2 - q3*q1))
                pitch = math.degrees(math.asin(
                    max(-1, min(1, 2*(q0*q2 - q3*q1)))
                ))
                print(
                    f"{ts} : 記録 #{record_count} "
                    f"EF=({data['Force_X']:.3f},{data['Force_Y']:.3f},{data['Force_Z']:.3f}) "
                    f"Roll={roll:.2f}°, Pitch={pitch:.2f}° "
                    f"Gain={data['Gain']:.2f}"
                )

        print("\n停止中…")
        master.close()
        print(f"総記録数: {record_count}")
        print("CSV ファイルを保存しました。")

if __name__ == "__main__":
    main()
