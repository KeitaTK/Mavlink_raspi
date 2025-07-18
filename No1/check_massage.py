from pymavlink import mavutil
import time
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
    # 元の設定で接続
    print("TELEM1通信接続（ハードウェアフロー制御有効）")
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=1000000, rtscts=True)

    master.wait_heartbeat(timeout=5)
    print(f"Heartbeat received from system {master.target_system}, component {master.target_component}")

    # 全てのデータストリームを要求
    for stream_id in [mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
                      mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
                      mavutil.mavlink.MAV_DATA_STREAM_EXTRA2,
                      mavutil.mavlink.MAV_DATA_STREAM_EXTRA3]:
        master.mav.request_data_stream_send(
            master.target_system,
            master.target_component,
            stream_id,
            1,  # 1Hz
            1   # start
        )

    print("全メッセージの受信を開始...")
    print("-" * 50)
    
    while running:
        msg = master.recv_match(blocking=True, timeout=1)
        
        if msg is not None:
            current_time = datetime.now().strftime("%Y/%m/%d %H:%M:%S")
            msg_type = msg.get_type()
            
            if msg_type == 'STATUSTEXT':
                text = msg.text.strip()
                print(f"{current_time} : {text}")
            
            # デバッグ用：すべてのメッセージタイプを表示
            elif msg_type not in ['HEARTBEAT', 'SYSTEM_TIME', 'TIMESYNC']:
                print(f"{current_time} : [DEBUG] {msg_type}")
        
        time.sleep(0.01)

except Exception as e:
    print(f"エラー: {e}")

finally:
    try:
        if 'master' in locals():
            master.close()
    except:
        pass
    print("プログラム終了")
