from pymavlink import mavutil
import time

# 1) 接続
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
master.wait_heartbeat()

# 2) SET_MESSAGE_INTERVAL でストリームを要求
def set_message_interval(msg_id, hz):
    interval_us = int(1e6 / hz)
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,  # ← ここを使う
        0,
        msg_id,
        interval_us,  # マイクロ秒単位
        0,0,0,0,0
    )
    time.sleep(0.05)

# EKFステータスを 2Hz
set_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 2)
# ローカル位置を 5Hz
set_message_interval(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 5)

# 3) 受信ループ
print("Listening for EKF/Position for 10s…")
t0 = time.time()
while time.time() - t0 < 10:
    msg = master.recv_match(blocking=False)
    if not msg:
        time.sleep(0.01)
        continue

    if msg.get_type() == 'EKF_STATUS_REPORT':
        print(f"[EKF] pos_var={msg.pos_horiz_variance:.6f} vel_var={msg.velocity_variance:.6f}")
    elif msg.get_type() == 'LOCAL_POSITION_NED':
        print(f"[POS] N={msg.x:+6.3f} E={msg.y:+6.3f} D={msg.z:+6.3f}")

master.close()
