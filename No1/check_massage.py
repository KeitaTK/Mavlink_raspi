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
    print("システム詳細診断開始")
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=1000000, rtscts=True)

    master.wait_heartbeat(timeout=5)
    print(f"Heartbeat received from system {master.target_system}, component {master.target_component}")

    # 複数のデータストリームを要求
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
        10,  # 10Hz
        1    # start
    )

    # システム情報を要求
    master.mav.autopilot_version_request_send(
        master.target_system,
        master.target_component
    )

    print("システム詳細診断中...")
    print("=" * 70)
    
    error_intervals = []
    last_error_time = None
    system_info_received = False
    
    while running:
        msg = master.recv_match(blocking=True, timeout=1)
        
        if msg is not None:
            current_time = datetime.now().strftime("%Y/%m/%d %H:%M:%S")
            msg_type = msg.get_type()
            
            if msg_type == 'STATUSTEXT':
                text = msg.text.strip()
                
                if "Internal errors 0x100000" in text:
                    current_dt = datetime.now()
                    
                    print(f"{current_time} : [CRITICAL] {text}")
                    print(f"    └── エラー発生時刻: {current_dt}")
                    
                    if last_error_time:
                        interval = (current_dt - last_error_time).total_seconds()
                        error_intervals.append(interval)
                        print(f"    └── 前回エラーから: {interval:.1f}秒")
                        
                        # 間隔パターンの分析
                        if len(error_intervals) >= 3:
                            avg_interval = sum(error_intervals[-3:]) / 3
                            print(f"    └── 平均間隔: {avg_interval:.1f}秒")
                            
                            if abs(avg_interval - 18) < 2:
                                print(f"    └── パターン: 約18秒の定期エラー（定期チェック処理）")
                            elif abs(avg_interval - 30) < 2:
                                print(f"    └── パターン: 約30秒の定期エラー（PreArm処理）")
                    
                    last_error_time = current_dt
                    print(f"    └── 総エラー回数: {len(error_intervals) + 1}")
                    print("-" * 50)
                
                elif "PreArm:" in text or "Arm:" in text:
                    print(f"{current_time} : [INFO] {text}")
                    
                elif "Thrust:" in text:
                    print(f"{current_time} : [SUCCESS] {text}")
                    print("    └── システムが正常にアームされました！")
                    
            elif msg_type == 'AUTOPILOT_VERSION':
                if not system_info_received:
                    print(f"{current_time} : [SYSTEM INFO] システム情報:")
                    print(f"    └── Flight Stack: {msg.flight_sw_version}")
                    print(f"    └── Middleware: {msg.middleware_sw_version}")
                    print(f"    └── Board Version: {msg.board_version}")
                    print(f"    └── Vendor ID: {msg.vendor_id}")
                    print(f"    └── Product ID: {msg.product_id}")
                    print("-" * 50)
                    system_info_received = True
                    
            elif msg_type == 'SYS_STATUS':
                # システム状態の監視
                if hasattr(msg, 'errors_count1') and msg.errors_count1 > 0:
                    print(f"{current_time} : [WARNING] システムエラー数: {msg.errors_count1}")
                    
            elif msg_type == 'HEARTBEAT':
                # システム状態の変化を監視
                status_names = {
                    0: "UNINIT",
                    1: "BOOT", 
                    2: "CALIBRATING",
                    3: "STANDBY",
                    4: "ACTIVE",
                    5: "CRITICAL",
                    6: "EMERGENCY",
                    7: "POWEROFF"
                }
                
                status_name = status_names.get(msg.system_status, f"UNKNOWN({msg.system_status})")
                
                # 状態変化があった場合のみ表示
                if not hasattr(signal_handler, 'last_status') or signal_handler.last_status != msg.system_status:
                    print(f"{current_time} : [STATUS] システム状態: {status_name}")
                    signal_handler.last_status = msg.system_status

except Exception as e:
    print(f"エラー: {e}")

finally:
    try:
        if 'master' in locals():
            master.close()
    except:
        pass
    
    # 診断結果の表示
    print("\n" + "="*70)
    print("診断結果:")
    print(f"  総エラー回数: {len(error_intervals) + 1}")
    
    if error_intervals:
        avg_interval = sum(error_intervals) / len(error_intervals)
        print(f"  平均エラー間隔: {avg_interval:.1f}秒")
        print(f"  エラー間隔: {[f'{i:.1f}s' for i in error_intervals]}")
        
        if avg_interval < 20:
            print("  → 推定原因: 定期的なシステムチェック処理での異常")
        elif avg_interval < 35:
            print("  → 推定原因: PreArmチェック処理での異常")
        else:
            print("  → 推定原因: 不定期なシステム異常")
    
    print("\n推奨対処:")
    print("  1. ファームウェアの再インストール")
    print("  2. パラメータの完全リセット")
    print("  3. ハードウェアの点検（電源、配線、センサー）")
    print("診断終了")
