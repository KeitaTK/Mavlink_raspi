#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
check_local_position_velocity.py

LOCAL_POSITION_NED (位置＋速度) を 5Hz で要求し、
内部 EKF3 推定の位置と速度を表示して検証します。
"""

import time
from pymavlink import mavutil

# 接続ポートを実際のものに合わせて変更してください
PORT = '/dev/ttyACM0'   # USB-CDC ACM
# PORT = '/dev/ttyAMA0' # TELEM1 UART
BAUD = 115200

def main():
    master = mavutil.mavlink_connection(PORT, baud=BAUD)
    master.wait_heartbeat()
    print(f"Heartbeat from sysid={master.target_system}\n")

    # LOCAL_POSITION_NED を 5Hz で要求
    interval_us = int(1e6 / 5)
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
        interval_us,
        0,0,0,0,0
    )
    time.sleep(0.1)

    print("Listening for LOCAL_POSITION_NED messages (position + velocity)...\n")
    start = time.time()
    while time.time() - start < 10:
        msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
        if not msg:
            continue
        # 位置と速度を同時に表示
        print(f"[POS]  x={msg.x:+6.3f} m  y={msg.y:+6.3f} m  z={msg.z:+6.3f} m  "
              f"| [VEL] vx={msg.vx:+6.3f} m/s  vy={msg.vy:+6.3f} m/s  vz={msg.vz:+6.3f} m/s")
    master.close()

if __name__ == '__main__':
    main()

