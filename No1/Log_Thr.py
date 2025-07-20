from pymavlink import mavutil
import time
import signal
import sys
import csv
import os
from datetime import datetime

running = True
csv_writer = None
csv_file = None

def signal_handler(sig, frame):
    global running, csv_file
    print('\n終了中...')
    running = False
    if csv_file:
        csv_file.close()
        print("CSVファイルを保存しました")

signal.signal(signal.SIGINT, signal_handler)

def parse_thrust_data(text):
    """
    Thrust: M0:0.245N M1:0.238N M2:0.251N M3:0.243N
    の形式から推力値を抽出
    """
    try:
        # "Thrust: "以降を取得
        data_part = text.split("Thrust: ")[1]
        
        # 各モーターの値を抽出
        motors = {}
        parts = data_part.split(" ")
        
        for part in parts:
            if "M" in part and ":" in part:
                # M0:0.245N の形式から抽出
                motor_part = part.split(":")
                motor_id = motor_part[0]  # M0, M1, M2, M3
                thrust_value = float(motor_part[1].replace("N", ""))  # Nを削除して数値化
                motors[motor_id] = thrust_value
        
        return motors
    except Exception as e:
        print(f"推力データ解析エラー: {e}")
        return None

def create_csv_filename():
    """
    日時_Thr.csv形式のファイル名を生成
    """
    current_time = datetime.now()
    timestamp = current_time.strftime("%Y%m%d_%H%M%S")
    
    # ディレクトリ作成
    log_dir = os.path.expanduser("~/LOGS_Pixhawk6c")
    os.makedirs(log_dir, exist_ok=True)
    
    filename = f"{timestamp}_Thr.csv"
    filepath = os.path.join(log_dir, filename)
    
    return filepath

try:
    print("推力データ記録開始")
    
    # CSVファイル準備
    csv_filepath = create_csv_filename()
    print(f"記録先: {csv_filepath}")
    
    csv_file = open(csv_filepath, 'w', newline='', encoding='utf-8')
    csv_writer = csv.writer(csv_file)
    
    # CSVヘッダー書き込み
    csv_writer.writerow(['Timestamp', 'M0_Thrust_N', 'M1_Thrust_N', 'M2_Thrust_N', 'M3_Thrust_N'])
    csv_file.flush()
    
    # MAVLink接続
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=1000000, rtscts=True)
    master.wait_heartbeat(timeout=5)
    print(f"Heartbeat received from system {master.target_system}, component {master.target_component}")
    
    # データストリーム要求
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
        10,  # 10Hz
        1    # start
    )
    
    print("推力データ監視中... (Ctrl+Cで終了)")
    print("=" * 60)
    
    record_count = 0
    
    while running:
        msg = master.recv_match(blocking=True, timeout=1)
        
        if msg is not None:
            msg_type = msg.get_type()
            
            if msg_type == 'STATUSTEXT':
                text = msg.text.strip()
                
                # Thrustメッセージのみ処理
                if "Thrust:" in text:
                    current_time = datetime.now()
                    timestamp = current_time.strftime("%Y/%m/%d %H:%M:%S.%f")[:-3]  # ミリ秒まで
                    
                    # 推力データ解析
                    thrust_data = parse_thrust_data(text)
                    
                    if thrust_data:
                        # CSV書き込み
                        row = [
                            timestamp,
                            thrust_data.get('M0', 0.0),
                            thrust_data.get('M1', 0.0),
                            thrust_data.get('M2', 0.0),
                            thrust_data.get('M3', 0.0)
                        ]
                        csv_writer.writerow(row)
                        csv_file.flush()  # 即座にファイルに書き込み
                        
                        record_count += 1
                        
                        # コンソール表示
                        print(f"{timestamp} : 推力記録 #{record_count}")
                        print(f"  M0:{thrust_data.get('M0', 0.0):.3f}N  M1:{thrust_data.get('M1', 0.0):.3f}N  M2:{thrust_data.get('M2', 0.0):.3f}N  M3:{thrust_data.get('M3', 0.0):.3f}N")

except Exception as e:
    print(f"エラー: {e}")

finally:
    try:
        if 'master' in locals():
            master.close()
    except:
        pass
    
    if csv_file:
        csv_file.close()
    
    print(f"\n記録終了")
    print(f"総記録数: {record_count}")
    print(f"保存先: {csv_filepath}")
    print("プログラム終了")
