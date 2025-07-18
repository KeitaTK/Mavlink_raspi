# from pymavlink import mavutil
# import time
# import signal
# import sys
# from datetime import datetime

# running = True

# def signal_handler(sig, frame):
#     global running
#     print('\n終了中...')
#     running = False

# # Ctrl+C で安全に終了するためのシグナルハンドラ
# signal.signal(signal.SIGINT, signal_handler)

# try:
#     # 1. シリアル接続の確立
#     print("TELEM1通信接続（ハードウェアフロー制御有効）")
#     master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=1000000, rtscts=True)

#     # 2. heartbeat 受信待ち
#     master.wait_heartbeat(timeout=5)
#     print(f"Heartbeat received from system {master.target_system}, component {master.target_component}")

#     # 3. STATUSTEXTメッセージの受信を要求
#     print("STATUSTEXTメッセージの受信を要求中...")
#     master.mav.request_data_stream_send(
#         master.target_system,
#         master.target_component,
#         mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
#         10,  # 10Hz（高頻度で要求）
#         1    # start
#     )

#     # 4. 全てのデータストリームを要求（念のため）
#     stream_types = [
#         mavutil.mavlink.MAV_DATA_STREAM_ALL,
#         mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
#         mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
#         mavutil.mavlink.MAV_DATA_STREAM_EXTRA2,
#         mavutil.mavlink.MAV_DATA_STREAM_EXTRA3
#     ]
    
#     for stream_type in stream_types:
#         master.mav.request_data_stream_send(
#             master.target_system,
#             master.target_component,
#             stream_type,
#             1,  # 1Hz
#             1   # start
#         )
#         time.sleep(0.1)

#     print("STATUSTEXTメッセージの受信を開始... (Ctrl+C で停止)")
#     print("=" * 60)
    
#     # 5. 継続的にメッセージを受信
#     while running:
#         msg = master.recv_match(blocking=True, timeout=1)
        
#         if msg is not None:
#             current_time = datetime.now().strftime("%Y/%m/%d %H:%M:%S")
#             msg_type = msg.get_type()
            
#             if msg_type == 'STATUSTEXT':
#                 # STATUSTEXTメッセージの詳細を表示
#                 text = msg.text.strip()
#                 severity = msg.severity
                
#                 # 重要度レベルを文字列に変換
#                 severity_text = {
#                     0: "EMERGENCY",
#                     1: "ALERT", 
#                     2: "CRITICAL",
#                     3: "ERROR",
#                     4: "WARNING",
#                     5: "NOTICE",
#                     6: "INFO",
#                     7: "DEBUG"
#                 }.get(severity, f"UNKNOWN({severity})")
                
#                 # Thrustメッセージかどうかを判定
#                 if "Thrust:" in text:
#                     print(f"{current_time} : [THRUST-{severity_text}] {text}")
#                 elif "PreArm:" in text:
#                     print(f"{current_time} : [PREARM-{severity_text}] {text}")
#                 elif "AP_Observer:" in text:
#                     print(f"{current_time} : [OBSERVER-{severity_text}] {text}")
#                 else:
#                     print(f"{current_time} : [STATUS-{severity_text}] {text}")
                    
#             # その他のメッセージは表示しない（STATUSTEXTのみに集中）
            
#         # CPU負荷軽減のため少し待機
#         time.sleep(0.01)

# except KeyboardInterrupt:
#     print("\nユーザーによって中断されました")
    
# except Exception as e:
#     print(f"通信エラー: {e}")

# finally:
#     try:
#         if 'master' in locals():
#             master.close()
#     except:
#         pass
#     print("プログラムを終了しました")


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
    print("PreArmエラー監視開始")
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=1000000, rtscts=True)

    master.wait_heartbeat(timeout=5)
    print(f"Heartbeat received from system {master.target_system}, component {master.target_component}")

    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
        10,  # 10Hz
        1    # start
    )

    print("PreArmエラーの詳細監視中...")
    print("=" * 60)
    
    error_count = 0
    last_error_time = None
    
    while running:
        msg = master.recv_match(blocking=True, timeout=1)
        
        if msg is not None:
            current_time = datetime.now().strftime("%Y/%m/%d %H:%M:%S")
            msg_type = msg.get_type()
            
            if msg_type == 'STATUSTEXT':
                text = msg.text.strip()
                
                if "PreArm:" in text:
                    error_count += 1
                    
                    if "Internal errors" in text:
                        print(f"{current_time} : [CRITICAL] {text}")
                        print(f"    └── エラー回数: {error_count}")
                        
                        # エラーコードの解析
                        if "0x100000" in text:
                            print(f"    └── エラータイプ: 制御フロー異常")
                            print(f"    └── 推奨対処: システム再起動")
                        
                        # 連続エラーの検出
                        if last_error_time:
                            interval = (datetime.now() - last_error_time).total_seconds()
                            print(f"    └── 前回エラーから: {interval:.1f}秒")
                        
                        last_error_time = datetime.now()
                        print("-" * 40)
                    
                    else:
                        print(f"{current_time} : [WARNING] {text}")
                
                elif "Internal errors" in text:
                    print(f"{current_time} : [SYSTEM] {text}")

except Exception as e:
    print(f"エラー: {e}")

finally:
    try:
        if 'master' in locals():
            master.close()
    except:
        pass
    print(f"総エラー回数: {error_count}")
    print("監視終了")
