#!/usr/bin/env python3
"""
ホームポジション設定と高度測定専用ツール
機体を手で持ち上げて高度変化をリアルタイム確認
"""

import time
from pymavlink import mavutil

# 接続設定
CONNECTION_PORT = '/dev/ttyAMA0'
BAUD_RATE = 115200

def main():
    """メイン処理"""
    print("=== Home Position & Altitude Test Tool ===")
    print("This tool will:")
    print("1. Set current position as home")
    print("2. Monitor altitude changes in real-time")
    print("3. You can lift the aircraft by hand to test altitude measurement")
    print()
    
    # 接続
    print(f"Connecting to vehicle on {CONNECTION_PORT}...")
    master = mavutil.mavlink_connection(CONNECTION_PORT, baud=BAUD_RATE, wait_heartbeat=True)
    print(f"✓ Connected! System {master.target_system} Component {master.target_component}")
    
    # 必要なメッセージ要求
    print("Requesting position messages...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        33, 500000, 0, 0, 0, 0, 0  # GLOBAL_POSITION_INT, 0.5秒間隔
    )
    
    time.sleep(2)
    
    # 設定前の高度確認
    print("\n--- BEFORE Home Position Setting ---")
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
    if msg:
        before_relative = msg.relative_alt / 1000.0
        before_absolute = msg.alt / 1000.0
        print(f"Relative altitude: {before_relative:.3f} m")
        print(f"Absolute altitude: {before_absolute:.3f} m")
    else:
        print("✗ Could not get position data")
        return
    
    # ホームポジション設定
    print("\n--- Setting Home Position ---")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_HOME, 0,
        1, 0, 0, 0, 0, 0, 0  # param1=1: use current location
    )
    print("✓ Home position set command sent")
    
    # コマンド確認
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    if ack_msg and ack_msg.command == mavutil.mavlink.MAV_CMD_DO_SET_HOME:
        if ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("✓ Home position set command ACCEPTED")
        else:
            print(f"⚠ Command result: {ack_msg.result}")
    
    time.sleep(3)  # 設定処理完了待機
    
    # 設定後の高度確認
    print("\n--- AFTER Home Position Setting ---")
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
    if msg:
        after_relative = msg.relative_alt / 1000.0
        after_absolute = msg.alt / 1000.0
        print(f"Relative altitude: {after_relative:.3f} m")
        print(f"Absolute altitude: {after_absolute:.3f} m")
        print(f"Relative altitude change: {after_relative - before_relative:.3f} m")
    else:
        print("✗ Could not get position data")
        return
    
    # リアルタイム高度監視
    print("\n" + "="*50)
    print("REAL-TIME ALTITUDE MONITORING")
    print("="*50)
    print("✓ Home position is now set as 0.000m reference")
    print("✓ You can now lift the aircraft by hand to test altitude measurement")
    print("✓ Press Ctrl+C to stop monitoring")
    print()
    print("Time     | Relative Alt | Absolute Alt | Change")
    print("---------+--------------+--------------+--------")
    
    try:
        start_time = time.time()
        baseline_relative = after_relative
        
        while True:
            msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
            if msg:
                current_time = time.time() - start_time
                current_relative = msg.relative_alt / 1000.0
                current_absolute = msg.alt / 1000.0
                height_change = current_relative - baseline_relative
                
                print(f"{current_time:7.1f}s | {current_relative:10.3f}m | {current_absolute:10.3f}m | {height_change:+6.3f}m")
            else:
                print("× No position data")
            
            time.sleep(0.5)
    
    except KeyboardInterrupt:
        print("\n\n✓ Monitoring stopped by user")
    
    print("\nTest completed!")

if __name__ == "__main__":
    main()
