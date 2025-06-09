from pymavlink import mavutil
import time

# 1) Pixhawk に接続
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
master.wait_heartbeat()

# 2) LOCAL_POSITION_NED を 5Hz で出すように要求
interval_us = int(1e6 / 5)  # 5Hz
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    0,
    mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
    interval_us, 0,0,0,0,0
)

time.sleep(0.1)  # クロック合わせ

print("→ LOCAL_POSITION_NED stream requested @5Hz")
