from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)

# カスタムメッセージ送信
master.mav.taki_pos_motive_send(
    master.target_system,
    master.target_component,
    1.5,  # x_coord
    2.3,  # y_coord
    3.7   # z_coord
)
print("TAKI_POS_MOTIVE sent: x=1.5, y=2.3, z=3.7")

# デバッグメッセージ監視
start_time = time.time()
while time.time() - start_time < 10:
    msg = master.recv_match(blocking=False)
    if msg:
        if msg.get_type() == 'STATUSTEXT':
            print(f"DEBUG: {msg.text}")
        elif msg.get_type() == 'HEARTBEAT':
            continue
    time.sleep(0.1)
