#!/usr/bin/env python3
from pymavlink import mavutil
import time

# 接続設定
PORT = '/dev/ttyACM0'
BAUD = 115200

master = mavutil.mavlink_connection(PORT, baud=BAUD)
master.wait_heartbeat()
print("Heartbeat received")

# EKF_STATUS_REPORT と LOCAL_POSITION_NED を 5Hz で要求
for msg_id, hz in [
    (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 5),
    (mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT,  2),
]:
    interval = int(1e6/hz)
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0, msg_id, interval, 0,0,0,0,0
    )
    time.sleep(0.05)

print("Listening for 10s…\n")
start = time.time()
while time.time() - start < 10:
    msg = master.recv_match(blocking=False)
    if not msg:
        time.sleep(0.01)
        continue

    if msg.get_type() == 'EKF_STATUS_REPORT':
        print(f"[EKF] pos_var={msg.pos_horiz_variance:.6f} vel_var={msg.velocity_variance:.6f}")

    elif msg.get_type() == 'LOCAL_POSITION_NED':
        # 位置と速度を同時に表示
        print(f"[POS]  N={msg.x:+6.3f} E={msg.y:+6.3f} D={msg.z:+6.3f}  "
              f"Vx={msg.vx:+5.2f} Vy={msg.vy:+5.2f} Vz={msg.vz:+5.2f}")

master.close()
