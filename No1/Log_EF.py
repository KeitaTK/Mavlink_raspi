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
file_closed = False  # ファイル状態管理フラグ

def signal_handler(sig, frame):
    global running
    print('\n終了中...')
    running = False  # ファイルクローズはfinally節で行う

signal.signal(signal.SIGINT, signal_handler)

def parse_force_data(text):
    """
    外力データの形式対応:
    FORCE: X=0.123 Y=-0.456 Z=0.789
    の形式から外力値を抽出
    """
    try:
        # "FORCE: "以降を取得
        if "FORCE:" in text:
            data_part = text.split("FORCE: ")[1]
        else:
            return None
        
        # X, Y, Z軸の値を抽出
        forces = {}
        parts = data_part.split(" ")
        
        for part in parts:
            if "=" in part:
                # X=0.123 の形式から抽出
                axis_part = part.split("=")
                axis = axis_part[0].strip()  # X, Y, Z
                force_value = float(axis_part[1].strip())
                forces[axis] = force_value
        
        return forces
    except Exception as e:
        print(f"外力データ解析エラー: {e}")
        return None

def create_csv_filename():
    """
    日時_EF.csv形式のファイル名を生成（External Force）
    """
    current_time = datetime.now()
    timestamp = current_time.strftime("%Y%m%d_%H%M%S")
    
    # ディレクトリ作成
    log_dir = os.path.expanduser("~/LOGS_Pixhawk6c")
    os.makedirs(log_dir, exist_ok=True)
    
    filename = f"{timestamp}_EF.csv"
    filepath = os.path.join(log_dir, filename)
    
    return filepath

def safe_write_csv(writer, file_handle, row):
    """安全なCSV書き込み"""
    global file_closed
    try:
        if not file_closed and file_handle and not file_handle.closed:
            writer.writerow(row)
            file_handle.flush()
            return True
        else:
            print("ファイルが閉じられているため書き込みをスキップ")
            return False
    except Exception as e:
        print(f"CSV書き込みエラー: {e}")
        return False

try:
    print("外力データ記録開始（AP_Observer対応版）")
    
    # CSVファイル準備
    csv_filepath = create_csv_filename()
    print(f"記録先: {csv_filepath}")
    
    csv_file = open(csv_filepath, 'w', newline='', encoding='utf-8')
    csv_writer = csv.writer(csv_file)
    
    # CSVヘッダー書き込み
    csv_writer.writerow(['Timestamp', 'Force_X_N', 'Force_Y_N', 'Force_Z_N', 'Force_Magnitude_N'])
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
    
    print("外力データ監視中... (Ctrl+Cで終了)")
    print("AP_Observerからの外力メッセージを待機中...")
    print("=" * 60)
    
    record_count = 0
    
    while running:
        try:
            msg = master.recv_match(blocking=True, timeout=1)
            
            if msg is not None:
                msg_type = msg.get_type()
                
                if msg_type == 'STATUSTEXT':
                    text = msg.text.strip()
                    
                    # FORCEメッセージのみ処理
                    if "FORCE:" in text:
                        current_time = datetime.now()
                        timestamp = current_time.strftime("%Y/%m/%d %H:%M:%S.%f")[:-3]  # ミリ秒まで
                        
                        # 外力データ解析
                        force_data = parse_force_data(text)
                        
                        if force_data:
                            # 外力の大きさ計算
                            force_x = force_data.get('X', 0.0)
                            force_y = force_data.get('Y', 0.0)
                            force_z = force_data.get('Z', 0.0)
                            
                            force_magnitude = (force_x**2 + force_y**2 + force_z**2)**0.5
                            
                            # CSV書き込み
                            row = [
                                timestamp,
                                force_x,
                                force_y,
                                force_z,
                                force_magnitude
                            ]
                            
                            # 安全な書き込み
                            if safe_write_csv(csv_writer, csv_file, row):
                                record_count += 1
                                
                                # コンソール表示
                                print(f"{timestamp} : 外力記録 #{record_count}")
                                print(f"  X:{force_x:.3f}N  Y:{force_y:.3f}N  Z:{force_z:.3f}N")
                                print(f"  大きさ: {force_magnitude:.3f}N")
        
        except Exception as e:
            print(f"データ処理エラー: {e}")
            continue

except Exception as e:
    print(f"初期化エラー: {e}")

finally:
    # 安全なクリーンアップ
    print(f"\n記録終了処理中...")
    
    try:
        if 'master' in locals():
            master.close()
    except:
        pass
    
    # ファイルクローズの安全な処理
    if csv_file and not csv_file.closed:
        try:
            file_closed = True  # フラグ設定
            csv_file.close()
            print("CSVファイルを正常に保存しました")
        except Exception as e:
            print(f"ファイルクローズエラー: {e}")
    
    print(f"総記録数: {record_count}")
    print(f"保存先: {csv_filepath}")
    print("プログラム終了")