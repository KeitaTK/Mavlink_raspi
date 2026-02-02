from pymavlink import mavutil
import signal
import sys
from datetime import datetime

running = True

def signal_handler(sig, frame):
    global running
    print('\n終了中...')
    running = False

signal.signal(signal.SIGINT, signal_handler)

try:
    print("MAVLink LOITER_OBS_POSメッセージ受信プログラム開始")
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=1000000, rtscts=True)

    master.wait_heartbeat(timeout=5)
    print(f"Heartbeat received from system {master.target_system}, component {master.target_component}")
    print("LOITER_OBS_POSメッセージ受信中... (x, y, z を表示)")
    print("=" * 70)
    
    while running:
        msg = master.recv_match(blocking=True, timeout=1)
        if msg is None:
            continue
        if msg.get_type() == 'STATUSTEXT':
            text = msg.text.decode('utf-8', errors='ignore') if isinstance(msg.text, bytes) else msg.text
            if text.startswith('LOITER_OBS_POS='):
                # テキストから数値をパース
                parts = text.split('=')[-1].split(',')
                if len(parts) == 3:
                    try:
                        x, y, z = map(float, parts)
                        current_time = datetime.now().strftime("%Y/%m/%d %H:%M:%S")
                        print(f"{current_time} : x={x:.6f}, y={y:.6f}, z={z:.6f}")
                        print("-" * 50)
                    except ValueError:
                        print(f"パース失敗: {text}")

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
