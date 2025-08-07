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

# signal.signal(signal.SIGINT, signal_handler)

# try:
#     print("システム詳細診断開始（AP_Observer対応版）")
#     master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=1000000, rtscts=True)

#     master.wait_heartbeat(timeout=5)
#     print(f"Heartbeat received from system {master.target_system}, component {master.target_component}")

#     # 複数のデータストリームを要求
#     master.mav.request_data_stream_send(
#         master.target_system,
#         master.target_component,
#         mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
#         10,  # 10Hz
#         1    # start
#     )

#     # システム情報を要求
#     master.mav.autopilot_version_request_send(
#         master.target_system,
#         master.target_component
#     )

#     print("システム詳細診断中... (AP_Observer推力データ監視)")
#     print("=" * 70)
    
#     error_intervals = []
#     last_error_time = None
#     system_info_received = False
#     thrust_message_count = 0
    
#     while running:
#         msg = master.recv_match(blocking=True, timeout=1)
        
#         if msg is not None:
#             current_time = datetime.now().strftime("%Y/%m/%d %H:%M:%S")
#             msg_type = msg.get_type()
            
#             if msg_type == 'STATUSTEXT':
#                 text = msg.text.strip()
                
#                 # Internal Errorsの処理
#                 if "Internal errors 0x100000" in text:
#                     current_dt = datetime.now()
                    
#                     print(f"{current_time} : [CRITICAL] {text}")
#                     print(f"    └── エラー発生時刻: {current_dt}")
                    
#                     if last_error_time:
#                         interval = (current_dt - last_error_time).total_seconds()
#                         error_intervals.append(interval)
#                         print(f"    └── 前回エラーから: {interval:.1f}秒")
                        
#                         # 間隔パターンの分析
#                         if len(error_intervals) >= 3:
#                             avg_interval = sum(error_intervals[-3:]) / 3
#                             print(f"    └── 平均間隔: {avg_interval:.1f}秒")
                            
#                             if abs(avg_interval - 18) < 2:
#                                 print(f"    └── パターン: 約18秒の定期エラー（定期チェック処理）")
#                             elif abs(avg_interval - 30) < 2:
#                                 print(f"    └── パターン: 約30秒の定期エラー（PreArm処理）")
                    
#                     last_error_time = current_dt
#                     print(f"    └── 総エラー回数: {len(error_intervals) + 1}")
#                     print("-" * 50)
                
#                 # PreArm/Armメッセージの処理
#                 elif "PreArm:" in text or "Arm:" in text:
#                     print(f"{current_time} : [INFO] {text}")
                
#                 # === AP_Observer関連メッセージの処理 ===
                
#                 # 推力データの処理
#                 elif "Thrust:" in text:
#                     thrust_message_count += 1
#                     print(f"{current_time} : [THRUST] {text}")
#                     print(f"    └── 推力データ取得成功 (#{thrust_message_count})")
                
#                 # モーターマスク情報の処理
#                 elif "Motor mask:" in text:
#                     print(f"{current_time} : [MOTOR_DEBUG] {text}")
#                     # マスク値の解析
#                     if "0x0F00" in text:
#                         print(f"    └── モーター8-11が有効 (SERVO9-12対応)")
#                     elif "0x000F" in text:
#                         print(f"    └── モーター0-3が有効 (SERVO1-4対応)")
#                     else:
#                         mask_value = text.split("0x")[1] if "0x" in text else "不明"
#                         print(f"    └── カスタムモーター設定: 0x{mask_value}")
                
#                 # PWM出力値の処理
#                 elif "PWM M" in text:
#                     print(f"{current_time} : [PWM_DEBUG] {text}")
#                     # PWM値の分析
#                     if "->" in text:
#                         try:
#                             parts = text.split("->")
#                             pwm_part = parts[0].split(":")[-1].strip()
#                             normalized_part = parts[1].strip()
#                             pwm_value = int(pwm_part)
                            
#                             if pwm_value > 1800:
#                                 print(f"    └── 高出力: {pwm_value}μs")
#                             elif pwm_value > 1500:
#                                 print(f"    └── 中出力: {pwm_value}μs")
#                             elif pwm_value > 1100:
#                                 print(f"    └── 低出力: {pwm_value}μs")
#                             else:
#                                 print(f"    └── 最小出力: {pwm_value}μs")
#                         except:
#                             print(f"    └── PWM解析中")
                
#                 # Raw推力値の処理
#                 elif "Raw M" in text:
#                     print(f"{current_time} : [RAW_DEBUG] {text}")
#                     # Raw値の状態分析
#                     if "0.0000" in text:
#                         print(f"    └── 推力推定システム未動作")
#                     else:
#                         print(f"    └── 推力推定システム動作中")
                
#                 # AP_Observer nullptrエラーの処理
#                 elif "AP_Observer: motors is nullptr!" in text:
#                     print(f"{current_time} : [ERROR] {text}")
#                     print(f"    └── モーターシステム初期化失敗")
                
#                 # その他のAP_Observer関連メッセージ
#                 elif "AP_Observer" in text:
#                     print(f"{current_time} : [OBSERVER] {text}")
                    
#             elif msg_type == 'AUTOPILOT_VERSION':
#                 if not system_info_received:
#                     print(f"{current_time} : [SYSTEM INFO] システム情報:")
#                     print(f"    └── Flight Stack: {msg.flight_sw_version}")
#                     print(f"    └── Middleware: {msg.middleware_sw_version}")
#                     print(f"    └── Board Version: {msg.board_version}")
#                     print(f"    └── Vendor ID: {msg.vendor_id}")
#                     print(f"    └── Product ID: {msg.product_id}")
#                     print("-" * 50)
#                     system_info_received = True
                    
#             elif msg_type == 'SYS_STATUS':
#                 # システム状態の監視
#                 if hasattr(msg, 'errors_count1') and msg.errors_count1 > 0:
#                     print(f"{current_time} : [WARNING] システムエラー数: {msg.errors_count1}")
                    
#             elif msg_type == 'HEARTBEAT':
#                 # システム状態の変化を監視
#                 status_names = {
#                     0: "UNINIT",
#                     1: "BOOT", 
#                     2: "CALIBRATING",
#                     3: "STANDBY",
#                     4: "ACTIVE",
#                     5: "CRITICAL",
#                     6: "EMERGENCY",
#                     7: "POWEROFF"
#                 }
                
#                 status_name = status_names.get(msg.system_status, f"UNKNOWN({msg.system_status})")
                
#                 # 状態変化があった場合のみ表示
#                 if not hasattr(signal_handler, 'last_status') or signal_handler.last_status != msg.system_status:
#                     print(f"{current_time} : [STATUS] システム状態: {status_name}")
#                     signal_handler.last_status = msg.system_status

# except Exception as e:
#     print(f"エラー: {e}")

# finally:
#     try:
#         if 'master' in locals():
#             master.close()
#     except:
#         pass
    
#     # 診断結果の表示
#     print("\n" + "="*70)
#     print("診断結果:")
#     print(f"  総エラー回数: {len(error_intervals) + 1 if error_intervals else 0}")
#     print(f"  推力データ受信数: {thrust_message_count}")
    
#     if error_intervals:
#         avg_interval = sum(error_intervals) / len(error_intervals)
#         print(f"  平均エラー間隔: {avg_interval:.1f}秒")
#         print(f"  エラー間隔: {[f'{i:.1f}s' for i in error_intervals]}")
        
#         if avg_interval < 20:
#             print("  → 推定原因: 定期的なシステムチェック処理での異常")
#         elif avg_interval < 35:
#             print("  → 推定原因: PreArmチェック処理での異常")
#         else:
#             print("  → 推定原因: 不定期なシステム異常")
    
#     print("\n推奨対処:")
#     print("  1. ファームウェアの再インストール")
#     print("  2. パラメータの完全リセット")
#     print("  3. ハードウェアの点検（電源、配線、センサー）")
    
#     if thrust_message_count > 0:
#         print("  4. AP_Observer推力データ正常受信 ✓")
#     else:
#         print("  4. AP_Observer推力データ未受信 - スケジューラー設定確認")
    
#     print("診断終了")




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
    print("MAVLinkメッセージ受信プログラム開始")
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=1000000, rtscts=True)

    master.wait_heartbeat(timeout=5)
    print(f"Heartbeat received from system {master.target_system}, component {master.target_component}")

    # データストリームを要求（元のプログラムを参考）
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_ALL,  # すべてのストリームを要求
        10,  # 10Hz
        1    # start
    )

    # システム情報を要求（元のプログラムを参考）
    master.mav.autopilot_version_request_send(
        master.target_system,
        master.target_component
    )

    print("メッセージ受信中... (すべてのメッセージを表示)")
    print("=" * 70)
    
    while running:
        msg = master.recv_match(blocking=True, timeout=1)
        
        if msg is not None:
            current_time = datetime.now().strftime("%Y/%m/%d %H:%M:%S")
            msg_type = msg.get_type()
            print(f"{current_time} : [MSG] Type: {msg_type}")
            print(f"    └── Content: {msg}")
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
