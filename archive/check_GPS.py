# import time
# import sys
# import math
# import pyned2lla
# from pymavlink import mavutil

# CONNECTION_PORT = '/dev/ttyAMA0'
# BAUD_RATE = 115200
# TAKEOFF_ALTITUDE = 0.5
# MOVE_DISTANCE = 0.02

# def connect_to_vehicle(port, baud):
#     """機体に接続"""
#     print(f"Connecting to vehicle on: {port} at {baud} baud")
#     master = mavutil.mavlink_connection(port, baud=baud, wait_heartbeat=True)
#     print(f"✓ Connected! System {master.target_system} Component {master.target_component}")
#     return master

# def request_gps_messages(master):
#     """最新の方法でGPSメッセージを要求"""
#     print("Requesting GPS messages using latest ArduPilot methods...")
    
#     # 方法1: SET_MESSAGE_INTERVAL コマンド（最新）
#     # GPS_RAW_INT (Message ID: 24) を1秒間隔で要求
#     master.mav.command_long_send(
#         master.target_system,
#         master.target_component,
#         mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
#         0,
#         24,        # GPS_RAW_INT message ID
#         1000000,   # 1秒間隔（マイクロ秒）
#         0, 0, 0, 0, 0
#     )
    
#     # GLOBAL_POSITION_INT (Message ID: 33) を1秒間隔で要求
#     master.mav.command_long_send(
#         master.target_system,
#         master.target_component,
#         mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
#         0,
#         33,        # GLOBAL_POSITION_INT message ID  
#         1000000,   # 1秒間隔（マイクロ秒）
#         0, 0, 0, 0, 0
#     )
    
#     # 方法2: REQUEST_DATA_STREAM（互換性のため）
#     master.mav.request_data_stream_send(
#         master.target_system,
#         master.target_component,
#         mavutil.mavlink.MAV_DATA_STREAM_POSITION,
#         2,  # 2Hz
#         1   # start
#     )
    
#     master.mav.request_data_stream_send(
#         master.target_system,
#         master.target_component,
#         mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,
#         2,  # 2Hz
#         1   # start
#     )
    
#     print("✓ GPS message requests sent")

# def wait_for_gps_fix(master, timeout=60):
#     """GPS Fix を待機（改良版）"""
#     print(f"Waiting for GPS fix (timeout: {timeout}s)...")
    
#     start_time = time.time()
#     while time.time() - start_time < timeout:
#         # GPS_RAW_INTメッセージを取得
#         gps_msg = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=2)
        
#         if gps_msg:
#             fix_type = gps_msg.fix_type
#             satellites = gps_msg.satellites_visible
#             hdop = gps_msg.eph / 100.0 if gps_msg.eph != 65535 else 99.99
            
#             fix_names = {0: "No GPS", 1: "No Fix", 2: "2D", 3: "3D", 4: "DGPS", 5: "RTK Float", 6: "RTK Fixed"}
#             fix_name = fix_names.get(fix_type, f"Unknown({fix_type})")
            
#             print(f"GPS Status: {fix_name} | Satellites: {satellites} | HDOP: {hdop:.2f}")
            
#             # 3D Fix以上かつ最低限の品質基準を満たしているかチェック
#             if fix_type >= 3 and satellites >= 6 and hdop <= 2.5:
#                 print("✓ GPS fix acquired with acceptable quality!")
#                 return True
#             elif fix_type >= 3:
#                 print("△ GPS fix acquired but quality may be marginal")
#                 return True
#         else:
#             print("× No GPS data received")
        
#         time.sleep(1)
    
#     print("✗ GPS fix timeout")
#     return False

# def get_home_position(master):
#     """ホームポジション取得（改良版）"""
#     print("Getting home position...")
    
#     # GPS Fix確認
#     if not wait_for_gps_fix(master):
#         return None
    
#     # 位置情報取得
#     for attempt in range(10):
#         pos_msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=3)
        
#         if pos_msg and pos_msg.lat != 0 and pos_msg.lon != 0:
#             lat_deg = pos_msg.lat / 1e7
#             lon_deg = pos_msg.lon / 1e7
#             alt_msl = pos_msg.alt / 1000.0
            
#             print(f"✓ Home position: Lat {lat_deg:.7f}°, Lon {lon_deg:.7f}°, Alt {alt_msl:.1f}m")
#             return pos_msg
        
#         print(f"Attempt {attempt + 1}/10: Waiting for valid position...")
#         time.sleep(1)
    
#     print("✗ Could not get valid position")
#     return None

# # 他の関数は前回と同じ...

# def main():
#     """メイン関数"""
#     master = connect_to_vehicle(CONNECTION_PORT, BAUD_RATE)
    
#     # 最新の方法でGPSメッセージを要求
#     request_gps_messages(master)
    
#     # 少し待機
#     time.sleep(2)
    
#     # ホームポジション取得
#     home_position = get_home_position(master)
#     if not home_position:
#         print("GPS acquisition failed. Exiting.")
#         return
    
#     print("GPS acquisition successful! Ready for flight operations.")

# if __name__ == "__main__":
#     main()

#!/usr/bin/env python3
"""
GUIDED モード対応 完全診断ツール
"""

import time
import sys
from pymavlink import mavutil

CONNECTION_PORT = '/dev/ttyAMA0'
BAUD_RATE = 115200

def connect_to_vehicle(port, baud):
    """機体に接続"""
    print(f"Connecting to vehicle on: {port} at {baud} baud")
    master = mavutil.mavlink_connection(port, baud=baud, wait_heartbeat=True)
    print(f"✓ Connected! System {master.target_system} Component {master.target_component}")
    return master

def request_all_guided_messages(master):
    """GUIDEDモードに必要な全メッセージを要求"""
    print("Requesting all GUIDED mode messages...")
    
    # 重要なメッセージを全て要求
    messages_to_request = [
        (1, "SYS_STATUS"),               # システム状態
        (24, "GPS_RAW_INT"),             # GPS生データ
        (33, "GLOBAL_POSITION_INT"),     # 全体位置情報
        (193, "EKF_STATUS_REPORT"),      # EKF状態レポート
        (242, "HOME_POSITION"),          # ホームポジション
        (253, "STATUSTEXT")              # ステータステキスト（エラーメッセージ）
    ]
    
    for msg_id, msg_name in messages_to_request:
        print(f"  Requesting {msg_name} (ID: {msg_id})")
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            msg_id, 1000000, 0, 0, 0, 0, 0  # 1秒間隔
        )
        time.sleep(0.1)  # 要求間隔
    
    # データストリームも要求（互換性のため）
    stream_requests = [
        (mavutil.mavlink.MAV_DATA_STREAM_POSITION, "POSITION"),
        (mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS, "RAW_SENSORS"),
        (mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS, "EXTENDED_STATUS")
    ]
    
    for stream_id, stream_name in stream_requests:
        print(f"  Requesting {stream_name} stream")
        master.mav.request_data_stream_send(
            master.target_system, master.target_component,
            stream_id, 2, 1  # 2Hz, start
        )
    
    print("✓ All message requests sent")

def set_home_position_explicitly(master):
    """ホームポジション明示的設定"""
    print("Setting home position explicitly...")
    
    # 現在位置をホームに設定
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME, 0,
        1, 0, 0, 0, 0, 0, 0  # 1=現在位置をホームに設定
    )
    
    # コマンド実行確認
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    if ack_msg and ack_msg.command == mavutil.mavlink.MAV_CMD_DO_SET_HOME:
        if ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("✓ Home position set command accepted")
        else:
            print(f"⚠ Home position set command result: {ack_msg.result}")
    else:
        print("⚠ No acknowledgment for home position set command")

def check_guided_arm_readiness_complete(master):
    """完全なGUIDEDモードアーム準備状況確認"""
    print(f"\n{'='*50}")
    print("COMPLETE GUIDED MODE ARM READINESS CHECK")
    print(f"{'='*50}")
    
    all_ready = True
    
    # 1. EKF状態の詳細確認
    print("1. EKF Status Check:")
    ekf_msg = master.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=5)
    if ekf_msg:
        # EKFフラグの詳細解析
        flags = ekf_msg.flags
        print(f"   EKF Flags: {bin(flags)} ({flags})")
        
        # 主要フラグの確認
        attitude_ok = bool(flags & (1 << 0))   # Attitude estimate OK
        velocity_ok = bool(flags & (1 << 1))   # Velocity estimate OK  
        position_ok = bool(flags & (1 << 2))   # Position estimate OK
        
        print(f"   Attitude OK: {attitude_ok}")
        print(f"   Velocity OK: {velocity_ok}")
        print(f"   Position OK: {position_ok}")
        
        # 分散値の確認
        print(f"   Position Variance: {ekf_msg.pos_horiz_variance:.3f}")
        print(f"   Velocity Variance: {ekf_msg.velocity_variance:.3f}")
        print(f"   Compass Variance: {ekf_msg.compass_variance:.3f}")
        
        if not (attitude_ok and velocity_ok and position_ok):
            print("   ✗ EKF not fully initialized")
            all_ready = False
        elif ekf_msg.pos_horiz_variance > 1.0:
            print("   ⚠ Position variance too high")
            all_ready = False
        else:
            print("   ✓ EKF OK")
    else:
        print("   ✗ No EKF status received")
        all_ready = False
    
    # 2. Home Position確認
    print("2. Home Position Check:")
    home_msg = master.recv_match(type='HOME_POSITION', blocking=True, timeout=5)
    if home_msg:
        print(f"   ✓ Home Position set")
        print(f"   Lat: {home_msg.latitude/1e7:.7f}")
        print(f"   Lon: {home_msg.longitude/1e7:.7f}")
        print(f"   Alt: {home_msg.altitude/1000.0:.2f}m")
    else:
        print("   ✗ No Home Position received")
        print("   Attempting to set home position...")
        set_home_position_explicitly(master)
        all_ready = False
    
    # 3. GPS詳細確認
    print("3. GPS Status Check:")
    gps_msg = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5)
    if gps_msg:
        fix_types = {0: "No GPS", 1: "No Fix", 2: "2D", 3: "3D", 4: "DGPS", 5: "RTK Float", 6: "RTK Fixed"}
        fix_name = fix_types.get(gps_msg.fix_type, f"Unknown({gps_msg.fix_type})")
        print(f"   GPS Fix: {fix_name}")
        print(f"   Satellites: {gps_msg.satellites_visible}")
        print(f"   HDOP: {gps_msg.eph/100.0:.2f}")
        
        if gps_msg.fix_type < 3:
            print("   ✗ GPS Fix insufficient for GUIDED mode")
            all_ready = False
        else:
            print("   ✓ GPS OK")
    else:
        print("   ✗ No GPS data received")
        all_ready = False
    
    # 4. システム状態確認
    print("4. System Status Check:")
    sys_msg = master.recv_match(type='SYS_STATUS', blocking=True, timeout=5)
    if sys_msg:
        # センサー健全性の確認
        sensors = sys_msg.onboard_control_sensors_health
        gps_healthy = bool(sensors & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS)
        gyro_healthy = bool(sensors & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO)
        accel_healthy = bool(sensors & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_ACCEL)
        mag_healthy = bool(sensors & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_MAG)
        
        print(f"   GPS Health: {'✓' if gps_healthy else '✗'}")
        print(f"   Gyroscope Health: {'✓' if gyro_healthy else '✗'}")
        print(f"   Accelerometer Health: {'✓' if accel_healthy else '✗'}")
        print(f"   Magnetometer Health: {'✓' if mag_healthy else '✗'}")
        
        # バッテリー状態
        voltage = sys_msg.voltage_battery / 1000.0
        print(f"   Battery Voltage: {voltage:.2f}V")
        
        if not all([gps_healthy, gyro_healthy, accel_healthy]):
            print("   ✗ Critical sensors unhealthy")
            all_ready = False
        else:
            print("   ✓ All critical sensors healthy")
    else:
        print("   ✗ No System Status received")
        all_ready = False
    
    # 5. Pre-arm エラーチェック
    print("5. Pre-arm Error Check:")
    print("   Monitoring for 5 seconds...")
    prearm_errors = []
    
    for i in range(50):  # 5秒間監視（100ms間隔）
        status_msg = master.recv_match(type='STATUSTEXT', blocking=False)
        if status_msg:
            text = status_msg.text.decode('utf-8') if isinstance(status_msg.text, bytes) else status_msg.text
            if 'PreArm' in text or 'prearm' in text:
                prearm_errors.append(text)
                print(f"   ⚠ {text}")
        time.sleep(0.1)
    
    if not prearm_errors:
        print("   ✓ No pre-arm errors detected")
    else:
        print(f"   ✗ {len(prearm_errors)} pre-arm error(s) found")
        all_ready = False
    
    print(f"\n{'='*50}")
    if all_ready:
        print("🎯 READY FOR GUIDED MODE ARMING!")
    else:
        print("❌ NOT READY - Fix issues above before arming")
    print(f"{'='*50}")
    
    return all_ready

def main():
    """メイン診断実行"""
    master = connect_to_vehicle(CONNECTION_PORT, BAUD_RATE)
    
    # 全メッセージ要求
    request_all_guided_messages(master)
    
    # メッセージが届くまで少し待機
    print("\nWaiting for messages to be established...")
    time.sleep(5)
    
    # 完全診断実行
    ready = check_guided_arm_readiness_complete(master)
    
    if not ready:
        print("\nRetrying in 10 seconds...")
        time.sleep(10)
        check_guided_arm_readiness_complete(master)

if __name__ == "__main__":
    main()
