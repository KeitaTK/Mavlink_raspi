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


import time
from pymavlink import mavutil

def check_guided_arm_readiness(master):
    """GUIDEDモードでのアーム準備状況を詳細確認"""
    print("\n=== GUIDED MODE ARM READINESS CHECK ===")
    
    # 1. EKF状態の確認
    print("1. Checking EKF status...")
    ekf_msg = master.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=5)
    if ekf_msg:
        print(f"   EKF flags: {bin(ekf_msg.flags)}")
        print(f"   EKF velocity variance: {ekf_msg.velocity_variance}")
        print(f"   EKF position variance: {ekf_msg.pos_horiz_variance}")
        print(f"   EKF compass variance: {ekf_msg.compass_variance}")
    else:
        print("   ✗ No EKF status received")
    
    # 2. Global Position状態の確認
    print("2. Checking Global Position...")
    global_pos = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
    if global_pos:
        print(f"   ✓ Global Position available")
        print(f"   Lat: {global_pos.lat/1e7:.7f}")
        print(f"   Lon: {global_pos.lon/1e7:.7f}")
        print(f"   Relative Alt: {global_pos.relative_alt/1000.0:.2f}m")
    else:
        print("   ✗ No Global Position received")
    
    # 3. Home Position設定状況の確認
    print("3. Checking Home Position...")
    home_pos = master.recv_match(type='HOME_POSITION', blocking=True, timeout=5)
    if home_pos:
        print(f"   ✓ Home Position set")
        print(f"   Home Lat: {home_pos.latitude/1e7:.7f}")
        print(f"   Home Lon: {home_pos.longitude/1e7:.7f}")
    else:
        print("   ⚠ No Home Position message (requesting set...)")
        # Home Position設定を明示的に要求
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_HOME, 0,
            1, 0, 0, 0, 0, 0, 0  # 1=現在位置をホームに設定
        )
    
    # 4. システム状態の確認
    print("4. Checking System Status...")
    sys_status = master.recv_match(type='SYS_STATUS', blocking=True, timeout=5)
    if sys_status:
        sensors = sys_status.onboard_control_sensors_health
        gps_ok = bool(sensors & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS)
        ahrs_ok = bool(sensors & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_3D_GYRO)
        print(f"   GPS Health: {'✓' if gps_ok else '✗'}")
        print(f"   AHRS Health: {'✓' if ahrs_ok else '✗'}")
    
    # 5. Pre-arm状態の確認（Mission Plannerメッセージ形式で表示）
    print("5. Waiting for any pre-arm messages...")
    for i in range(10):  # 10秒間メッセージを監視
        msg = master.recv_match(type='STATUSTEXT', blocking=False)
        if msg:
            text = msg.text.decode('utf-8') if isinstance(msg.text, bytes) else msg.text
            if 'PreArm' in text or 'prearm' in text:
                print(f"   ⚠ {text}")
        time.sleep(1)

def wait_for_ekf_and_position_ready(master, timeout=120):
    """EKFとPosition準備完了まで待機"""
    print(f"\nWaiting for EKF and Position initialization (timeout: {timeout}s)...")
    
    start_time = time.time()
    while time.time() - start_time < timeout:
        # Home Position設定の確認
        home_pos = master.recv_match(type='HOME_POSITION', blocking=False)
        global_pos = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        
        if home_pos and global_pos:
            print("✓ Both Home Position and Global Position are available")
            
            # EKFの状態も確認
            ekf_msg = master.recv_match(type='EKF_STATUS_REPORT', blocking=True, timeout=2)
            if ekf_msg:
                # EKFの主要な分散値をチェック
                pos_variance = ekf_msg.pos_horiz_variance
                vel_variance = ekf_msg.velocity_variance
                
                print(f"EKF Position Variance: {pos_variance:.3f}")
                print(f"EKF Velocity Variance: {vel_variance:.3f}")
                
                # 分散値が十分小さければOK（通常は1.0以下が望ましい）
                if pos_variance < 1.0 and vel_variance < 1.0:
                    print("✓ EKF converged! Ready for GUIDED mode arming.")
                    return True
            
            print("△ Positions available but EKF still converging...")
        
        time.sleep(2)
        
        # 30秒ごとに詳細状況を表示
        if int(time.time() - start_time) % 30 == 0:
            check_guided_arm_readiness(master)
    
    print("✗ EKF/Position initialization timeout")
    return False

# 使用例
def main():
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200, wait_heartbeat=True)
    
    # 詳細診断
    check_guided_arm_readiness(master)
    
    # EKF準備完了まで待機
    if wait_for_ekf_and_position_ready(master):
        print("🎯 Ready for GUIDED mode arming!")
    else:
        print("❌ Not ready for GUIDED mode arming")

if __name__ == "__main__":
    main()
