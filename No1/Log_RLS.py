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

def signal_handler(sig, frame):
    global running
    running = False
    print("\n終了信号受信, 停止します…")

signal.signal(signal.SIGINT, signal_handler)

def create_csv_filepath():
    now = datetime.now().strftime("%Y%m%d_%H%M%S")
    directory = os.path.expanduser("~/LOGS_Pixhawk6c")
    os.makedirs(directory, exist_ok=True)
    return os.path.join(directory, f"{now}_RLS_Observer.csv")

def parse_rls_message(text):
    """RLSメッセージをパースする"""
    text = text.strip()
    
    # Cold startメッセージ
    m = re.match(r"RLS: Cold start (\d+)/(\d+) time=(\d+)", text)
    if m:
        return {
            "type": "cold_start",
            "current_count": int(m.group(1)),
            "total_count": int(m.group(2)),
            "time_ms": int(m.group(3))
        }
    
    # RLS運用メッセージ
    m = re.match(
        r"RLS: F_curr=\[([-\d\.]+),([-\d\.]+),([-\d\.]+)\] "
        r"F_pred=\[([-\d\.]+),([-\d\.]+),([-\d\.]+)\] "
        r"Δt=([\d\.]+)ms time=(\d+)",
        text
    )
    if m:
        fx_curr, fy_curr, fz_curr = map(float, m.groups()[:3])
        fx_pred, fy_pred, fz_pred = map(float, m.groups()[3:6])
        delta_t = float(m.group(7))
        time_ms = int(m.group(8))
        
        # 外力の大きさを計算
        mag_curr = math.sqrt(fx_curr**2 + fy_curr**2 + fz_curr**2)
        mag_pred = math.sqrt(fx_pred**2 + fy_pred**2 + fz_pred**2)
        
        return {
            "type": "rls_running",
            "F_curr_X": fx_curr,
            "F_curr_Y": fy_curr,
            "F_curr_Z": fz_curr,
            "F_curr_Mag": mag_curr,
            "F_pred_X": fx_pred,
            "F_pred_Y": fy_pred,
            "F_pred_Z": fz_pred,
            "F_pred_Mag": mag_pred,
            "prediction_time_ms": delta_t,
            "time_ms": time_ms
        }
    
    return None

def main():
    csv_path = create_csv_filepath()
    with open(csv_path, "w", newline="", encoding="utf-8") as csvfile:
        writer = csv.writer(csvfile)
        writer.writerow([
            "Timestamp",
            "Message_Type",
            "F_curr_X_N", "F_curr_Y_N", "F_curr_Z_N", "F_curr_Magnitude_N",
            "F_pred_X_N", "F_pred_Y_N", "F_pred_Z_N", "F_pred_Magnitude_N",
            "Prediction_Time_ms",
            "Pixhawk_Time_ms",
            "Cold_Start_Progress",
            "Cold_Start_Total"
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
            
            # RLSメッセージかチェック
            if not msg.text.startswith("RLS:"):
                continue
            
            data = parse_rls_message(msg.text)
            if not data:
                continue

            # CSV 書き込み
            ts = datetime.now().strftime("%Y/%m/%d %H:%M:%S.%f")[:-3]
            
            if data["type"] == "cold_start":
                row = [
                    ts,
                    "Cold_Start",
                    "", "", "", "",  # F_curr
                    "", "", "", "",  # F_pred
                    "",              # Prediction_Time_ms
                    data["time_ms"],
                    data["current_count"],
                    data["total_count"]
                ]
                # コンソール表示
                print(f"{ts} : Cold Start 進捗 {data['current_count']}/{data['total_count']}")
                
            elif data["type"] == "rls_running":
                row = [
                    ts,
                    "RLS_Running",
                    data["F_curr_X"], data["F_curr_Y"], data["F_curr_Z"], data["F_curr_Mag"],
                    data["F_pred_X"], data["F_pred_Y"], data["F_pred_Z"], data["F_pred_Mag"],
                    data["prediction_time_ms"],
                    data["time_ms"],
                    "",  # Cold_Start_Progress
                    ""   # Cold_Start_Total
                ]
                # コンソール表示（5回に1回）
                if record_count % 5 == 0:
                    print(
                        f"{ts} : 記録 #{record_count} "
                        f"F_curr=({data['F_curr_X']:.3f},{data['F_curr_Y']:.3f},{data['F_curr_Z']:.3f}) "
                        f"F_pred=({data['F_pred_X']:.3f},{data['F_pred_Y']:.3f},{data['F_pred_Z']:.3f}) "
                        f"Δt={data['prediction_time_ms']:.1f}ms"
                    )
            
            writer.writerow(row)
            csvfile.flush()
            record_count += 1

        print("\n停止中…")
        master.close()
        print(f"総記録数: {record_count}")
        print("CSV ファイルを保存しました。")

if __name__ == "__main__":
    main()
