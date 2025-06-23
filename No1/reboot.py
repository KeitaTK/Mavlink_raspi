#!/usr/bin/env python3
"""
Pixhawkリブートのみ実行
"""

from pymavlink import mavutil
import time

def reboot_only():
    """リブートのみ実行"""
    
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
    master.wait_heartbeat()
    
    print("=== Pixhawk Reboot Only ===")
    print(f"Connected to System ID: {master.target_system}")
    
    # 検索結果[2]のMAVLinkコマンド246を使用
    print("Sending reboot command...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,  # Command 246
        0,
        1,  # param1=1: Reboot autopilot
        0,  # param2=0: Reserved
        0,  # param3=0: Reserved  
        0,  # param4=0: Reserved
        0,  # param5=0: Reserved
        0,  # param6=0: Reserved
        0   # param7=0: Reserved
    )
    
    print("✅ Reboot command sent")
    print("🔄 Pixhawk will reboot in 3-5 seconds")
    
    master.close()

if __name__ == "__main__":
    reboot_only()
