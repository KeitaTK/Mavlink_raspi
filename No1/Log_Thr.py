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

def parse_thrust_data(text):
    """
    新しい形式対応:
    Thrust: M0:0.533 M1:0.439 M2:0.654 M3:0.584
    の形式から推力値を抽出（単位Nなし）
    """
    try:
        # "Thrust: "以降を取得（[THRUST]タグがある場合も対応）
        if "Thrust:" in text:
            data_part = text.split("Thrust: ")[1]
        else:
            return None
        
        # 各モーターの値を抽出
        motors = {}
        parts = data_part.split(" ")
        
        for part in parts:
            if "M" in part and ":" in part:
                # M0:0.533 の形式から抽出（単位なし対応）
                motor_part = part.split(":")
                motor_id = motor_part[0]  # M0, M1, M2, M3
                
                # 単位Nがある場合とない場合の両方に対応
                thrust_str = motor_part[1].replace("N", "").strip()
                thrust_value = float(thrust_str)
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
    print("推力データ記録開始（AP_Observer対応版）")
    
    # CSVファイル準備
    csv_filepath = create_csv_filename()
    print(f"記録先: {csv_filepath}")
    
    csv_file = open(csv_filepath, 'w', newline='', encoding='utf-8')
    csv_writer = csv.writer(csv_file)
    
    # CSVヘッダー書き込み
    csv_writer.writerow(['Timestamp', 'M0_Thrust_N', 'M1_Thrust_N', 'M2_Thrust_N', 'M3_Thrust_N', 'Total_Thrust_N'])
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
    print("AP_Observerからの推力メッセージを待機中...")
    print("=" * 60)
    
    record_count = 0
    
    while running:
        try:
            msg = master.recv_match(blocking=True, timeout=1)
            
            if msg is not None:
                msg_type = msg.get_type()
                
                if msg_type == 'STATUSTEXT':
                    text = msg.text.strip()
                    
                    # Thrustメッセージのみ処理（PWMデバッグメッセージは無視）
                    if "Thrust:" in text and "PWM" not in text:
                        current_time = datetime.now()
                        timestamp = current_time.strftime("%Y/%m/%d %H:%M:%S.%f")[:-3]  # ミリ秒まで
                        
                        # 推力データ解析
                        thrust_data = parse_thrust_data(text)
                        
                        if thrust_data:
                            # 合計推力計算
                            total_thrust = (thrust_data.get('M0', 0.0) + 
                                          thrust_data.get('M1', 0.0) + 
                                          thrust_data.get('M2', 0.0) + 
                                          thrust_data.get('M3', 0.0))
                            
                            # CSV書き込み
                            row = [
                                timestamp,
                                thrust_data.get('M0', 0.0),
                                thrust_data.get('M1', 0.0),
                                thrust_data.get('M2', 0.0),
                                thrust_data.get('M3', 0.0),
                                total_thrust
                            ]
                            
                            # 安全な書き込み
                            if safe_write_csv(csv_writer, csv_file, row):
                                record_count += 1
                                
                                # コンソール表示
                                print(f"{timestamp} : 推力記録 #{record_count}")
                                print(f"  M0:{thrust_data.get('M0', 0.0):.3f}N  M1:{thrust_data.get('M1', 0.0):.3f}N  M2:{thrust_data.get('M2', 0.0):.3f}N  M3:{thrust_data.get('M3', 0.0):.3f}N")
                                print(f"  合計推力: {total_thrust:.3f}N")
        
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
