#!/usr/bin/env python3
"""
ArduPilot制御高度取得 - 最シンプル版
"""
import time
from pymavlink import mavutil

def get_control_altitude():
    """制御で使用される高度をリアルタイム取得"""
    
    # 接続
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200, wait_heartbeat=True)
    print("Connected!")
    
    # LOCAL_POSITION_NEDメッセージを要求[5]
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        32, 100000, 0, 0, 0, 0, 0  # 100ms間隔
    )
    
    print("Time    | Control Altitude")
    print("--------+-----------------")
    
    start_time = time.time()
    
    while True:
        # 制御高度取得
        msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
        
        if msg:
            # ArduPilot制御で使用される実際の高度[5]
            control_altitude = -msg.z  # NED座標系なので符号反転[5]
            
            elapsed = time.time() - start_time
            print(f"{elapsed:6.1f}s | {control_altitude:13.3f}m")
        
        time.sleep(0.1)

if __name__ == "__main__":
    get_control_altitude()
