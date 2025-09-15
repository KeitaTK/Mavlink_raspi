from pymavlink import mavutil
import signal
import sys
from datetime import datetime
import math

running = True

def signal_handler(sig, frame):
    global running
    print('\n終了中...')
    running = False

signal.signal(signal.SIGINT, signal_handler)

def quat_to_euler(q0, q1, q2, q3):
    # q0 = w, q1 = x, q2 = y, q3 = z
    # ロール (x軸回り)
    sinr_cosp = 2.0 * (q0 * q1 + q2 * q3)
    cosr_cosp = 1.0 - 2.0 * (q1 * q1 + q2 * q2)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    # ピッチ (y軸回り)
    sinp = 2.0 * (q0 * q2 - q3 * q1)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)
    else:
        pitch = math.asin(sinp)
    # ラジアン→度変換
    return math.degrees(roll), math.degrees(pitch)

try:
    print("MAVLinkデバッグメッセージ受信プログラム開始")
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=1000000, rtscts=True)

    master.wait_heartbeat(timeout=5)
    print(f"Heartbeat received from system {master.target_system}, component {master.target_component}")
    print("デバッグメッセージ受信中... (OBSV_UPD をロール/ピッチ度で表示)")
    print("=" * 70)
    
    while running:
        msg = master.recv_match(blocking=True, timeout=1)
        if msg is None:
            continue
        if msg.get_type() == 'STATUSTEXT':
            text = msg.text.decode('utf-8', errors='ignore') if isinstance(msg.text, bytes) else msg.text
            if text.startswith('OBSV_UPD'):
                # テキストから数値をパース
                parts = text.split('Q=')[-1].split(',')
                q0, q1, q2, q3 = map(float, parts)
                roll_deg, pitch_deg = quat_to_euler(q0, q1, q2, q3)
                current_time = datetime.now().strftime("%Y/%m/%d %H:%M:%S")
                print(f"{current_time} : Roll={roll_deg:.2f}°, Pitch={pitch_deg:.2f}°")
                print("-" * 50)
            elif text.startswith('OBS_pos='):
                # テキストから座標をパース
                try:
                    parts = text.split('OBS_pos=')[-1].split(',')
                    x, y, z = map(float, parts)
                    current_time = datetime.now().strftime("%Y/%m/%d %H:%M:%S")
                    print(f"{current_time} : X={x:.6f}, Y={y:.6f}, Z={z:.6f}")
                    print("-" * 50)
                except Exception as e:
                    print(f"OBS_posパースエラー: {e}")
            elif 'input_pos_NEU_cm called' in text:
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
