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

    # ※ このプログラムは input_pos_NEU_cm called のみを受信・表示します。
    #     OBS_pos=... の受信・表示は check_quateOBS.py で対応済みです。

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
            elif text.startswith('OBS_pos='):
                try:
                    parts = text.split('OBS_pos=')[-1].split(',')
                    x, y, z = map(float, parts)
                    current_time = datetime.now().strftime("%Y/%m/%d %H:%M:%S")
                    print(f"{current_time} : OBS_pos X={x:.6f}, Y={y:.6f}, Z={z:.6f}")
                    print("-" * 50)
                except Exception as e:
                    print(f"OBS_posパースエラー: {e}")
            elif text.startswith('OBS_accel='):
                try:
                    parts = text.split('OBS_accel=')[-1].split(',')
                    ax, ay, az = map(float, parts)
                    current_time = datetime.now().strftime("%Y/%m/%d %H:%M:%S")
                    print(f"{current_time} : OBS_accel X={ax:.6f}, Y={ay:.6f}, Z={az:.6f}")
                    print("-" * 50)
                except Exception as e:
                    print(f"OBS_accelパースエラー: {e}")

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
