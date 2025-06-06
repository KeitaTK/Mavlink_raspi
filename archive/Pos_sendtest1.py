from pymavlink import mavutil
import time

# 接続確立
# master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
master.wait_heartbeat()
print("Connected!")

# カスタムメッセージ送信
for i in range(100):
    master.mav.taki_pos_motive_send(1.5, 2.3, i)
print("TAKI_POS_MOTIVE sent: x=1.5, y=2.3, z=3.7")

# デバッグメッセージを5秒間監視
for i in range(100):  # 0.5秒間（0.1秒間隔）
    msg = master.recv_match(blocking=False)
    if msg and msg.get_type() == 'STATUSTEXT':
        print(f"DEBUG: {msg.text}")
    time.sleep(0.1)
