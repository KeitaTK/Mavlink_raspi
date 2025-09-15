from pymavlink import mavutil
import signal
from datetime import datetime

running = True

def signal_handler(sig, frame):
    global running
    print('\n終了中...')
    running = False

signal.signal(signal.SIGINT, signal_handler)

try:
    print("MAVLink input_pos_NEU_cmメッセージ受信プログラム開始")
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=1000000, rtscts=True)

    master.wait_heartbeat(timeout=5)
    print(f"Heartbeat received from system {master.target_system}, component {master.target_component}")
    print("input_pos_NEU_cm calledメッセージ受信中...")
    print("=" * 70)

    while running:
        msg = master.recv_match(blocking=True, timeout=1)
        if msg is None:
            continue
        if msg.get_type() == 'STATUSTEXT':
            text = msg.text.decode('utf-8', errors='ignore') if isinstance(msg.text, bytes) else msg.text
            if 'input_pos_NEU_cm called' in text:
                current_time = datetime.now().strftime("%Y/%m/%d %H:%M:%S")
                print(f"{current_time} : input_pos_NEU_cm called")
                print("-" * 50)

except Exception as e:
    print(f"エラー: {e}")

finally:
    try:
        if 'master' in locals():
            master.close()
    except:
        pass
    print("\n" + "=" * 70)
    print("プログラム終了")
