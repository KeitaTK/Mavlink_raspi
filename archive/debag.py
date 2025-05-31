from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)

# カスタムメッセージリクエスト送信
master.mav.taki_custome1_request_send(
    master.target_system,
    master.target_component
)
print("TAKI_CUSTOME1_REQUEST sent")

# 両方のメッセージタイプを監視
start_time = time.time()
while time.time() - start_time < 10:
    msg = master.recv_match(blocking=False)
    if msg:
        if msg.get_type() == 'STATUSTEXT':
            print(f"DEBUG: {msg.text}")
        elif msg.get_type() == 'TAKI_CUSTOME1':
            print(f"SUCCESS: TAKI_CUSTOME1 received - counter={msg.test_counter}")
        elif msg.get_type() == 'HEARTBEAT':
            continue  # ハートビートは無視
        else:
            print(f"Other message: {msg.get_type()}")
    time.sleep(0.1)
