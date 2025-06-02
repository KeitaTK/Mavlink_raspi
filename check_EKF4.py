from pymavlink import mavutil
import time

# 1. 接続
# USB接続の場合
print("USB接続通信")
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
# telem1で接続するとき
# print("tekem1通信接続")
# master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
master.wait_heartbeat()
print("Heartbeat received")

# 2. EKF／位置推定メッセージをリクエスト
def set_interval(msg_id, hz):
    interval_us = int(1e6 / hz)
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_GET_MESSAGE_INTERVAL,
        0,
        msg_id,
        interval_us,
        0,0,0,0,0
    )
    time.sleep(0.05)

#  2Hz で EKF_STATUS_REPORT
set_interval(mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 2)
#  5Hz で LOCAL_POSITION_NED
set_interval(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 5)


print("Listening for EKF output for 10 seconds…")
start = time.time()
while time.time() - start < 10:
    msg = master.recv_match(blocking=False)
    if not msg:
        time.sleep(0.01)
        continue

    if msg.get_type() == 'EKF_STATUS_REPORT':
        print(f"[EKF] pos_var={msg.pos_horiz_variance:.6f}, vel_var={msg.velocity_variance:.6f}")
    elif msg.get_type() == 'LOCAL_POSITION_NED':
        print(f"[POS] N={msg.x:+6.3f} E={msg.y:+6.3f} D={msg.z:+6.3f}")
