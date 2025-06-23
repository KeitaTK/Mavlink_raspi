#!/usr/bin/env python3
"""
EKF完全セットアップスクリプト
"""

from pymavlink import mavutil
import time

def complete_ekf_setup():
    """GPS原点設定 + EKFパラメータ設定"""
    
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
    master.wait_heartbeat()
    
    print("=== Complete EKF Setup ===")
    
    # Step 1: GPS原点設定
    print("1. Setting GPS origin...")
    lat = 35.6895
    lon = 139.6917  
    alt = 50
    
    master.mav.set_gps_global_origin_send(
        master.target_system,
        int(lat * 10000000),
        int(lon * 10000000),
        int(alt * 1000)
    )
    print(f"✅ GPS origin set: {lat}, {lon}, {alt}m")
    time.sleep(1)
    
    # Step 2: EKF3パラメータ設定
    print("2. Setting EKF3 parameters...")
    ekf_params = {
        'AHRS_EKF_TYPE': 3,
        'EK3_ENABLE': 1,
        'EK3_SRC1_POSXY': 6,
        'EK3_SRC1_POSZ': 6,
        'EK3_SRC1_YAW': 6,
    }
    
    for param, value in ekf_params.items():
        master.mav.param_set_send(
            master.target_system,
            master.target_component,
            param.encode('utf-8'),
            value,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
        time.sleep(0.3)
    
    print("✅ EKF3 parameters set")
    
    # Step 3: パラメータ保存
    print("3. Saving parameters...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    print("✅ Parameters saved")
    
    print("\n🔄 Please reboot Pixhawk before starting motion capture data")
    master.close()

if __name__ == "__main__":
    complete_ekf_setup()
