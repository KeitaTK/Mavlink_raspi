#!/usr/bin/env python3
"""
EKF強制リセット・再初期化
"""

from pymavlink import mavutil
import time

def force_ekf_reset():
    """EKFを強制リセット"""
    
    master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
    master.wait_heartbeat()
    
    print("=== Force EKF Reset ===")
    
    # 1. EKF原点リセット
    print("1. Resetting EKF origin...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        0,
        0, 0, 0, 0, 0, 0, 1  # EKF reset only
    )
    print("   EKF reset command sent")
    time.sleep(3)
    
    # 2. GPS原点強制設定
    print("2. Force setting GPS origin...")
    master.mav.set_gps_global_origin_send(
        master.target_system,
        int(35.6895 * 10000000),
        int(139.6917 * 10000000), 
        int(50 * 1000)
    )
    print("   GPS origin set")
    time.sleep(2)
    
    # 3. EKF手動初期化コマンド
    print("3. Manual EKF initialization...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_CALIBRATION,
        0,
        0, 0, 0, 0, 0, 0, 1  # EKF calibration
    )
    print("   EKF calibration command sent")
    time.sleep(2)
    
    print("4. Waiting for initialization...")
    print("   Please continue motion capture data transmission")
    
    master.close()

if __name__ == "__main__":
    force_ekf_reset()
