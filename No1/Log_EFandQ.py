from pymavlink import mavutil
import time
import signal
import sys
import csv
import os
from datetime import datetime
import re
from collections import deque

running = True
csv_writer = None
csv_file = None
file_closed = False
message_queue = deque(maxlen=10)  # 最新10メッセージを保持


def signal_handler(sig, frame):
    global running
    print('\n終了中...')
    running = False


signal.signal(signal.SIGINT, signal_handler)


def parse_force_and_quat(text):
    """
    外力と補正クオータニオンデータの形式対応:
    完全なCORRECTIONメッセージから値を抽出
    """
    try:
        force_match = re.search(r'Force=\[(.*?)\]', text)
        quat_match = re.search(r'Quat_RPY.*?=.*?\[(.*?)\]deg', text)
        
        if force_match and quat_match:
            force_str = force_match.group(1)
            quat_str = quat_match.group(1)
            
            force_vals = list(map(float, force_str.split(',')))
            quat_vals = list(map(float, quat_str.split(',')))
            
            return {
                'Force_X': force_vals[0],
                'Force_Y': force_vals[1],
                'Force_Z': force_vals[2],
                'Quat_Roll': quat_vals[0],
                'Quat_Pitch': quat_vals[1],
                'Quat_Yaw': quat_vals[2]
            }
        else:
            return None
    except Exception as e:
        return None


def reconstruct_correction_message(message_queue):
    """
    メッセージキューから完全なCORRECTIONメッセージを再構成
    """
    try:
        # キューを逆順（最新から古い順）で検索
        messages = list(message_queue)
        
        for i in range(len(messages)):
            current_msg = messages[i]
            
            # CORRECTIONで始まるメッセージを探す
            if "CORRECTION:" in current_msg and "Force=" in current_msg:
                force_part = current_msg
                
                # Quat_RPY部分の補完を試みる
                # パターン1: 既に完全なメッセージ
                if re.search(r'Quat_RPY.*?=.*?\[.*?\]deg', force_part):
                    return force_part
                
                # パターン2: Quat_RPY部分が不完全
                # 後続のメッセージでQuat値を探す
                for j in range(i+1, min(i+5, len(messages))):  # 最大4メッセージ後まで検索
                    next_msg = messages[j]
                    
                    # [数値,数値,数値]degパターンを探す
                    quat_match = re.search(r'\[([0-9\-\.,\s]+)\]deg', next_msg)
                    if quat_match:
                        # 完全なメッセージを再構成
                        if "Quat_RPY=" in force_part:
                            complete_msg = force_part + f"[{quat_match.group(1)}]deg"
                        elif "Quat_RPY" in force_part:
                            complete_msg = force_part + f"=[{quat_match.group(1)}]deg"
                        else:
                            complete_msg = force_part + f" Quat_RPY=[{quat_match.group(1)}]deg"
                        
                        return complete_msg
                
        return None
    except Exception as e:
        print(f"メッセージ再構成エラー: {e}")
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
    print("外力と補正クオータニオンデータ記録開始（高度分割メッセージ対応版）")
    
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
    print("高度分割メッセージ再構成システム稼働中...")
    print("=" * 60)
    
    record_count = 0
    last_process_time = 0
    
    while running:
        try:
            msg = master.recv_match(blocking=True, timeout=1)
            
            if msg is not None:
                msg_type = msg.get_type()
                
                if msg_type == 'STATUSTEXT':
                    text = msg.text.strip()
                    
                    # メッセージをキューに追加
                    message_queue.append(text)
                    
                    # 処理頻度制御（0.1秒ごとに処理）
                    current_time = time.time()
                    if current_time - last_process_time > 0.1:
                        last_process_time = current_time
                        
                        # 完全なCORRECTIONメッセージの再構成を試行
                        complete_message = reconstruct_correction_message(message_queue)
                        
                        if complete_message:
                            timestamp = datetime.now().strftime("%Y/%m/%d %H:%M:%S.%f")[:-3]
                            
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
                                    
                                    # コンソール表示（10回に1回）
                                    if record_count % 10 == 0:
                                        print(f"{timestamp} : データ記録 #{record_count}")
                                        print(f"  Force: X:{force_x:.3f}N  Y:{force_y:.3f}N  Z:{force_z:.3f}N")
                                        print(f"  Magnitude: {force_magnitude:.3f}N")
                                        print(f"  Quat_RPY: Roll:{data['Quat_Roll']:.2f}deg  Pitch:{data['Quat_Pitch']:.2f}deg  Yaw:{data['Quat_Yaw']:.2f}deg")
                                        print("-" * 60)
                                    elif record_count % 50 == 0:
                                        print(f"記録中... #{record_count}")
        
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
