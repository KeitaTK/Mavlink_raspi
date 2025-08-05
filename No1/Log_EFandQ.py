from pymavlink import mavutil
import time
import signal
import sys
import csv
import os
from datetime import datetime
import re

running = True
csv_writer = None
csv_file = None
file_closed = False
message_buffer = ""
expecting_quat_line = False


def signal_handler(sig, frame):
    global running
    print('\n終了中...')
    running = False


signal.signal(signal.SIGINT, signal_handler)


def parse_force_and_quat(text):
    """
    外力と補正クオータニオンデータの解析
    CORRECTION: Force=[x,y,z] Quat_RPY=[roll,pitch,yaw]deg 形式
    """
    try:
        force_match = re.search(r'Force=\[(.*?)\]', text)
        quat_match = re.search(r'Quat_RPY=\[(.*?)\]deg', text)

        if not force_match or not quat_match:
            return None

        force_vals = list(map(float, force_match.group(1).split(',')))
        quat_vals = list(map(float, quat_match.group(1).split(',')))

        return {
            'Force_X': force_vals[0],
            'Force_Y': force_vals[1],
            'Force_Z': force_vals[2],
            'Quat_Roll': quat_vals[0],
            'Quat_Pitch': quat_vals[1],
            'Quat_Yaw': quat_vals[2]
        }
    except Exception as e:
        return None


def process_correction_message(text):
    """
    分割されたCORRECTIONメッセージを処理
    """
    global message_buffer, expecting_quat_line
    
    text = text.strip()
    
    # CORRECTIONで始まり Quat_RPY=[ で終わる場合（分割の第1部）
    if text.startswith('CORRECTION:') and text.endswith('Quat_RPY=['):
        message_buffer = text
        expecting_quat_line = True
        return None
    
    # 前の行がCORRECTION分割第1部で、今の行がクオータニオン値の場合
    elif expecting_quat_line and re.match(r'^[0-9\-\.\,\s]+\]deg$', text):
        complete_message = message_buffer + text
        message_buffer = ""
        expecting_quat_line = False
        return complete_message
    
    # その他は無視またはリセット
    else:
        message_buffer = ""
        expecting_quat_line = False
        return None


def create_csv_filename():
    """
    日時_EFandQ.csv形式のファイル名を生成
    """
    current_time = datetime.now()
    timestamp = current_time.strftime("%Y%m%d_%H%M%S")
    
    log_dir = os.path.expanduser("~/LOGS_Pixhawk6c")
    os.makedirs(log_dir, exist_ok=True)
    
    filename = f"{timestamp}_EFandQ.csv"
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
            return False
    except Exception as e:
        print(f"CSV書き込みエラー: {e}")
        return False


try:
    print("外力と補正クオータニオンデータ記録開始（シンプル分割対応版）")
    
    # CSVファイル準備
    csv_filepath = create_csv_filename()
    print(f"記録先: {csv_filepath}")
    
    csv_file = open(csv_filepath, 'w', newline='', encoding='utf-8')
    csv_writer = csv.writer(csv_file)
    
    # CSVヘッダー書き込み
    csv_writer.writerow(['Timestamp', 'Force_X_N', 'Force_Y_N', 'Force_Z_N', 'Force_Magnitude_N', 
                         'Quat_Roll_deg', 'Quat_Pitch_deg', 'Quat_Yaw_deg'])
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
    
    print("外力と補正クオータニオンデータ監視中... (Ctrl+Cで終了)")
    print("2行分割メッセージ結合システム稼働中...")
    print("=" * 60)
    
    record_count = 0
    
    while running:
        try:
            msg = master.recv_match(blocking=True, timeout=1)
            
            if msg is not None:
                msg_type = msg.get_type()
                
                if msg_type == 'STATUSTEXT':
                    text = msg.text.strip()
                    
                    # 分割メッセージを処理
                    complete_message = process_correction_message(text)
                    
                    if complete_message:
                        current_time = datetime.now()
                        timestamp = current_time.strftime("%Y/%m/%d %H:%M:%S.%f")[:-3]
                        
                        # データ解析
                        data = parse_force_and_quat(complete_message)
                        
                        if data:
                            # 外力の大きさ計算
                            force_x = data['Force_X']
                            force_y = data['Force_Y']
                            force_z = data['Force_Z']
                            
                            force_magnitude = (force_x**2 + force_y**2 + force_z**2)**0.5
                            
                            # CSV書き込み
                            row = [
                                timestamp,
                                force_x,
                                force_y,
                                force_z,
                                force_magnitude,
                                data['Quat_Roll'],
                                data['Quat_Pitch'],
                                data['Quat_Yaw']
                            ]
                            
                            # 安全な書き込み
                            if safe_write_csv(csv_writer, csv_file, row):
                                record_count += 1
                                
                                # コンソール表示（5回に1回）
                                if record_count == 1 or record_count % 5 == 0:
                                    print(f"{timestamp} : データ記録 #{record_count}")
                                    print(f"  Force: X:{force_x:.3f}N  Y:{force_y:.3f}N  Z:{force_z:.3f}N")
                                    print(f"  Magnitude: {force_magnitude:.3f}N")
                                    print(f"  Quat_RPY: Roll:{data['Quat_Roll']:.2f}deg  Pitch:{data['Quat_Pitch']:.2f}deg  Yaw:{data['Quat_Yaw']:.2f}deg")
                                    print("-" * 60)
                                elif record_count % 20 == 0:
                                    print(f"記録継続中... #{record_count}")
        
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
            file_closed = True
            csv_file.close()
            print("CSVファイルを正常に保存しました")
        except Exception as e:
            print(f"ファイルクローズエラー: {e}")
    
    print(f"総記録数: {record_count}")
    print(f"保存先: {csv_filepath}")
    print("プログラム終了")
