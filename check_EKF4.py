#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_stream_methods.py

・request_data_stream_send(MAV_DATA_STREAM_POSITION) → LOCAL_POSITION_NED
・MAV_CMD_GET_MESSAGE_INTERVAL → LOCAL_POSITION_NED / EKF_STATUS_REPORT
を同時に要求し、
10秒間の受信状況を確認します。
"""

import time
from pymavlink import mavutil

# 接続設定（必要に応じて変更）
PORT     = '/dev/ttyACM0'   # USB-CDC ACM
# PORT   = '/dev/ttyAMA0'   # UART TELEM1
BAUD     = 115200
DURATION = 10.0             # 受信監視時間（秒）

def main():
    # 1) 接続
    master = mavutil.mavlink_connection(PORT, baud=BAUD)
    master.wait_heartbeat()
    print(f"Heartbeat received (sysid={master.target_system})\n")

    # 2) ストリーム要求①：旧 API
    print("→ request_data_stream_send: POSITION @5Hz")
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION,  # LOCAL_POSITION_NED を含む
        5,  # Hz
        1   # start
    )
    time.sleep(0.1)

    # 3) ストリーム要求②：GET_MESSAGE_INTERVAL (MAVLink2)
    def set_interval(msg_id, hz):
        interval_us = int(1e6 / hz)
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_GET_MESSAGE_INTERVAL,
            0,
            msg_id,
            interval_us,
            0, 0, 0, 0, 0
        )
        time.sleep(0.05)

    print("→ GET_MESSAGE_INTERVAL: LOCAL_POSITION_NED @5Hz")
    set_interval(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 5)

    print("→ GET_MESSAGE_INTERVAL: EKF_STATUS_REPORT @2Hz")
    set_interval(mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 2)

    print("\nListening for EKF/Position for 10s…\n")

    # 4) 受信ループ
    start = time.time()
    while time.time() - start < DURATION:
        msg = master.recv_match(blocking=False)
        if not msg:
            time.sleep(0.01)
            continue

        mtype = msg.get_type()
        if mtype == 'LOCAL_POSITION_NED':
            print(f"[POS]  N={msg.x:+6.3f} E={msg.y:+6.3f} D={msg.z:+6.3f} "
                  f"Vx={msg.vx:+5.2f} Vy={msg.vy:+5.2f} Vz={msg.vz:+5.2f}")
        elif mtype == 'EKF_STATUS_REPORT':
            print(f"[EKF] vel_var={msg.velocity_variance:.6f} "
                  f"pos_var={msg.pos_horiz_variance:.6f}")
    master.close()

if __name__ == '__main__':
    main()
