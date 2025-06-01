#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_position.py

EKFのローカル位置（LOCAL_POSITION_NED）と
グローバル位置（GLOBAL_POSITION_INT）が取得できているか確認するスクリプト
"""

import time
from pymavlink import mavutil

# 設定
SERIAL_PORT = '/dev/ttyACM0'   # USB経由でEKF監視する場合
BAUDRATE    = 115200
DURATION    = 10.0             # 監視時間 (秒)
LOCAL_RATE  = 5    # LOCAL_POSITION_NED のレート(Hz)
GLOBAL_RATE = 1    # GLOBAL_POSITION_INT のレート(Hz)

def main():
    print("Connecting to autopilot...")
    master = mavutil.mavlink_connection(SERIAL_PORT, baud=BAUDRATE)
    master.wait_heartbeat()
    print(f"Heartbeat from sysid={master.target_system} compid={master.target_component}\n")

    # メッセージストリーム要求
    def request_stream(msg_id, hz):
        interval = int(1e6 / hz)
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_GET_MESSAGE_INTERVAL,
            0,
            msg_id,
            interval,
            0,0,0,0,0
        )

    print(f"Requesting LOCAL_POSITION_NED @ {LOCAL_RATE}Hz")
    request_stream(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, LOCAL_RATE)
    time.sleep(0.1)
    print(f"Requesting GLOBAL_POSITION_INT @ {GLOBAL_RATE}Hz")
    request_stream(mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, GLOBAL_RATE)
    time.sleep(0.1)

    # 監視ループ
    print(f"\nListening for {DURATION:.1f}s...")
    start = time.time()

    local_count  = 0
    global_count = 0
    first_local  = True
    first_global = True
    last_local   = None
    last_global  = None

    while time.time() - start < DURATION:
        msg = master.recv_match(blocking=False)
        if not msg:
            time.sleep(0.01)
            continue

        t = time.time() - start
        if msg.get_type() == 'LOCAL_POSITION_NED':
            local_count += 1
            last_local = msg
            if first_local:
                print(f"\n[{t:.2f}s] LOCAL_POSITION_NED:")
                print(f"  N={msg.x:+7.3f} m  E={msg.y:+7.3f} m  D={msg.z:+7.3f} m")
                print(f"  V_N={msg.vx:+6.2f} m/s  V_E={msg.vy:+6.2f} m/s  V_D={msg.vz:+6.2f} m/s")
                first_local = False

        elif msg.get_type() == 'GLOBAL_POSITION_INT':
            global_count += 1
            last_global = msg
            if first_global:
                lat = msg.lat / 1e7
                lon = msg.lon / 1e7
                alt = msg.alt / 1000.0
                rel = msg.relative_alt / 1000.0
                print(f"\n[{t:.2f}s] GLOBAL_POSITION_INT:")
                print(f"  LAT={lat:.7f}  LON={lon:.7f}")
                print(f"  ALT={alt:.2f} m  REL_ALT={rel:.2f} m")
                first_global = False

    # 結果サマリー
    print("\n" + "="*50)
    print("Summary:")
    print(f" LOCAL_POSITION_NED messages: {local_count}  ({local_count/DURATION:.1f} Hz)")
    print(f" GLOBAL_POSITION_INT messages: {global_count}  ({global_count/DURATION:.1f} Hz)")

    if last_local:
        print(" Last LOCAL_POSITION_NED:")
        print(f"  N={last_local.x:+7.3f}  E={last_local.y:+7.3f}  D={last_local.z:+7.3f}")
    if last_global:
        lat = last_global.lat / 1e7
        lon = last_global.lon / 1e7
        alt = last_global.alt / 1000.0
        print(" Last GLOBAL_POSITION_INT:")
        print(f"  LAT={lat:.7f}  LON={lon:.7f}  ALT={alt:.2f} m")

    # 判定
    ok = True
    if local_count == 0:
        print("\n❌ No LOCAL_POSITION_NED received")
        ok = False
    if global_count == 0:
        print("\n⚠️ No GLOBAL_POSITION_INT received (normal if no GPS)")
        # GPSを使わない場合は警告のみ

    if ok and local_count > 0:
        print("\n✅ Position information is being received correctly!")
    else:
        print("\n🔧 Please check EKF settings and data sources")

    master.close()

if __name__ == '__main__':
    main()
