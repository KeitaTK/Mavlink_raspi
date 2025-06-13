#!/usr/bin/env python3
from pymavlink import mavutil

# ----- 接続設定 -----
PORT = '/dev/ttyACM0'      # 例：UART
BAUD = 115200

# ----- MAVLink 接続 -----
master = mavutil.mavlink_connection(PORT, baud=BAUD)
master.wait_heartbeat()
print(f'Heartbeat OK  sys={master.target_system} comp={master.target_component}')

# ----- EKF 推定位置 (#33) を 5 Hz で要求 -----
MSG_ID = 33                       # GLOBAL_POSITION_INT
PERIOD_US = 200_000               # 200 ms = 5 Hz
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    0,                # confirmation
    MSG_ID,           # param1: メッセージID
    PERIOD_US,        # param2: 周期 [µs]
    0,0,0,0,0)        # 以降 unused

# ----- 受信ループ -----
try:
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        if msg:
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.relative_alt / 1000.0
            print(f'EKF Pos  Lat:{lat:.7f}  Lon:{lon:.7f}  Alt:{alt:.3f} m')
except KeyboardInterrupt:
    print('Stop')
