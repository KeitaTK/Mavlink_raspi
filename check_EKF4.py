#!/usr/bin/env python3
from pymavlink import mavutil
import time

print("USB接続通信")
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
# telem1で接続するとき
# print("tekem1通信接続")
# master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
master.wait_heartbeat()
print("heartbeat")

# ストリーム要求（GET_MESSAGE_INTERVAL方式）
for msg_id, hz in [
    (mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 2),
    (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 5),
]:
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_GET_MESSAGE_INTERVAL,
        0, msg_id, int(1e6/hz), 0,0,0,0,0
    )
    time.sleep(0.05)

print("Listening...")
t0 = time.time()
while time.time() - t0 < 10:
    msg = master.recv_match(blocking=False)
    if not msg:
        time.sleep(0.01)
        continue
    if msg.get_type() == 'EKF_STATUS_REPORT':
        print(f"[EKF] pos_var={msg.pos_horiz_variance:.4f}")
    if msg.get_type() == 'LOCAL_POSITION_NED':
        print(f"[POS] N={msg.x:.2f} E={msg.y:.2f} D={msg.z:.2f}")
