from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
master.wait_heartbeat()
print("Heartbeat received")

# TAKI_CUSTOME1リクエスト送信（メッセージIDや関数名は実装に合わせてください）
master.mav.taki_custome1_request_send(
    master.target_system,
    master.target_component
)
print("Request sent")

print("Waiting for STATUSTEXT logs... (Ctrl+Cで終了)\n")
try:
    while True:
        msg = master.recv_match(type='STATUSTEXT', blocking=True, timeout=5)
        if msg:
            # デコード
            text = msg.text.decode('utf-8', errors='ignore') if isinstance(msg.text, (bytes, bytearray)) else msg.text
            # "TAKI_EKF" を含む行のみ表示
            if "TAKI_EKF" in text:
                print(f"[STATUSTEXT] {text}")
except KeyboardInterrupt:
    print("\nStopped.")
