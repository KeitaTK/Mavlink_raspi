#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from pymavlink import mavutil
import time

PORT    = '/dev/ttyACM0'
BAUD    = 115200
DUR     = 10.0

def main():
    # 接続＆ハートビート待ち
    master = mavutil.mavlink_connection(PORT, baud=BAUD)
    master.wait_heartbeat()
    print(f"Heartbeat from sysid={master.target_system}")

    # データストリーム要求
    streams = [
        (mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,    50),
        (mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,2),
        (mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,         10),
        (mavutil.mavlink.MAV_DATA_STREAM_POSITION,       5),
    ]
    for sid, hz in streams:
        master.mav.request_data_stream_send(
            master.target_system,
            master.target_component,
            sid, hz, 1
        )
        time.sleep(0.1)

    # メッセージ受信＆カウント
    print(f"Listening for {DUR}s…")
    counts = {}
    start = time.time()
    while time.time() - start < DUR:
        msg = master.recv_match(blocking=False)
        if msg:
            t = msg.get_type()
            counts[t] = counts.get(t, 0) + 1
        time.sleep(0.01)

    # 結果表示
    print("\n=== Message Counts ===")
    for t, c in sorted(counts.items()):
        print(f"{t:20s}: {c:3d} msgs ({c/DUR:.1f} Hz)")

    # 判定
    if counts.get('HIGHRES_IMU',0) == 0:
        print("❌ No HIGHRES_IMU data")
    if counts.get('ATTITUDE',0) == 0:
        print("❌ No ATTITUDE data")
    if counts.get('SYS_STATUS',0) == 0:
        print("❌ No SYS_STATUS data")
    if counts.get('EKF_STATUS_REPORT',0) == 0:
        print("❌ No EKF_STATUS_REPORT data")
    if counts.get('LOCAL_POSITION_NED',0) == 0:
        print("❌ No LOCAL_POSITION_NED data")
    else:
        print("✅ EKF pipeline is working correctly")

    master.close()

if __name__ == '__main__':
    main()
