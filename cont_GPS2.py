#!/usr/bin/env python3
"""
ArduPilot精密制御システム - 最小限版
GPS_INPUT使用、自動初期化、2cm精度制御
"""

import time
import sys
import tty
import termios
import math
import pyned2lla
from pymavlink import mavutil

# --- 設定項目 ---
CONNECTION_PORT = '/dev/ttyAMA0'
BAUD_RATE = 115200
TAKEOFF_ALTITUDE = 0.1  # 10cm離陸
MOVE_DISTANCE = 0.02    # 2cm移動

def get_key():
    """キー入力取得"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
        if ch == '\x1b':
            ch2 = sys.stdin.read(1)
            ch3 = sys.stdin.read(1)
            if ch2 == '[':
                if ch3 == 'A': return 'up'
                if ch3 == 'B': return 'down'
                if ch3 == 'C': return 'right'
                if ch3 == 'D': return 'left'
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def connect_to_vehicle(port, baud):
    """機体に接続"""
    print(f"Connecting to vehicle on: {port} at {baud} baud")
    master = mavutil.mavlink_connection(port, baud=baud, wait_heartbeat=True)
    print(f"✓ Connected! System {master.target_system} Component {master.target_component}")
    return master

def setup_minimal_messages(master):
    """最小限のメッセージ要求"""
    print("Setting up essential messages...")
    
    # GPS情報のみ要求
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        24, 1000000, 0, 0, 0, 0, 0  # GPS_RAW_INT, 1秒間隔
    )
    
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        33, 1000000, 0, 0, 0, 0, 0  # GLOBAL_POSITION_INT, 1秒間隔
    )
    
    print("✓ Message setup complete")

def wait_for_gps_fix(master):
    """GPS Fix待機（EKF原点自動設定）"""
    print("Waiting for GPS fix (EKF origin will be set automatically)...")
    
    for attempt in range(60):  # 60秒間待機
        gps_msg = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=2)
        if gps_msg and gps_msg.fix_type >= 3:
            print(f"✓ GPS Fix: {gps_msg.fix_type}, Satellites: {gps_msg.satellites_visible}")
            print("✓ EKF origin automatically set")
            
            # 位置データ確認
            pos_msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=3)
            if pos_msg and pos_msg.lat != 0:
                lat_deg = pos_msg.lat / 1e7
                lon_deg = pos_msg.lon / 1e7
                print(f"✓ Position: {lat_deg:.7f}, {lon_deg:.7f}")
                return pos_msg
        
        print(f"  Waiting... ({attempt + 1}/60)")
        time.sleep(1)
    
    print("✗ GPS fix timeout")
    return None

def wait_for_guided_mode(master):
    """GUIDEDモード待機"""
    print("Waiting for GUIDED mode...")
    while True:
        msg = master.recv_match(type='HEARTBEAT', blocking=True)
        if msg.custom_mode == 4:  # ArduCopterのGUIDEDモード
            print("✓ GUIDED mode active")
            break
        time.sleep(0.5)

def wait_for_arm(master):
    """アーム待機（ホームポジション自動設定）"""
    print("Waiting for vehicle to be armed...")
    print("Please arm using transmitter: Throttle down + Yaw right for 5 seconds")
    
    while True:
        heartbeat = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if heartbeat:
            armed = bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if armed:
                print("✓ Vehicle is ARMED!")
                print("✓ Home position automatically set")
                return True
        time.sleep(0.5)

def takeoff(master, altitude):
    """離陸"""
    print(f"Taking off to {altitude}m...")
    
    # 離陸コマンド送信
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, altitude
    )
    
    # 高度監視
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=3)
        if msg:
            current_alt = msg.relative_alt / 1000.0
            print(f"Altitude: {current_alt:.3f}m")
            
            if current_alt >= altitude * 0.95:
                print("✓ Target altitude reached")
                break
        time.sleep(0.5)

def move_to_position(master, lat_int, lon_int, altitude):
    """指定GPS座標へ移動"""
    master.mav.set_position_target_global_int_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,
        lat_int, lon_int, altitude,
        0, 0, 0, 0, 0, 0, 0, 0
    )

def precision_control(master, start_position):
    """精密制御ループ"""
    print("\n=== PRECISION CONTROL MODE ===")
    print("Controls:")
    print("  ↑ : North  | ↓ : South")
    print("  ← : West   | → : East")
    print("  q : Quit")
    print("Movement distance: 2cm per key press")
    print("Ready for control...")
    
    # 初期化
    wgs84 = pyned2lla.wgs84()
    lat0_deg = start_position.lat / 1e7
    lon0_deg = start_position.lon / 1e7
    alt0_msl = start_position.alt / 1000.0
    lat0_rad = math.radians(lat0_deg)
    lon0_rad = math.radians(lon0_deg)
    
    # 累積移動量
    total_north = 0.0
    total_east = 0.0
    
    while True:
        key = get_key()
        moved = False
        
        if key == 'q':
            print("Exiting precision control")
            break
        elif key == 'up':
            total_north += MOVE_DISTANCE
            moved = True
            print(f"→ North +{MOVE_DISTANCE*100:.0f}cm (total: {total_north*100:+.0f}cm)")
        elif key == 'down':
            total_north -= MOVE_DISTANCE
            moved = True
            print(f"→ South +{MOVE_DISTANCE*100:.0f}cm (total: {total_north*100:+.0f}cm)")
        elif key == 'right':
            total_east += MOVE_DISTANCE
            moved = True
            print(f"→ East +{MOVE_DISTANCE*100:.0f}cm (total: {total_east*100:+.0f}cm)")
        elif key == 'left':
            total_east -= MOVE_DISTANCE
            moved = True
            print(f"→ West +{MOVE_DISTANCE*100:.0f}cm (total: {total_east*100:+.0f}cm)")
        
        if moved:
            # NED座標からGPS座標に変換
            (target_lat_rad, target_lon_rad, _) = pyned2lla.ned2lla(
                lat0_rad, lon0_rad, alt0_msl, 
                total_north, total_east, 0, wgs84
            )
            
            # 整数値に変換して送信
            target_lat_int = int(math.degrees(target_lat_rad) * 1e7)
            target_lon_int = int(math.degrees(target_lon_rad) * 1e7)
            
            # 移動コマンド送信
            move_to_position(master, target_lat_int, target_lon_int, TAKEOFF_ALTITUDE)
            
            time.sleep(0.1)  # コマンド処理時間

def main():
    """メイン処理"""
    print("ArduPilot Precision Control System - Minimal Version")
    print("=" * 55)
    
    try:
        import pyned2lla
    except ImportError:
        print("Error: pyned2lla not found. Install with: pip install pyned2lla")
        sys.exit(1)
    
    # 1. 接続
    master = connect_to_vehicle(CONNECTION_PORT, BAUD_RATE)
    
    # 2. メッセージ設定
    setup_minimal_messages(master)
    time.sleep(2)
    
    print("\n=== STARTUP SEQUENCE ===")
    print("1. Switch to GUIDED mode on transmitter")
    print("2. Wait for GPS fix and initialization")
    print("3. Arm the vehicle")
    print("4. Automatic takeoff and precision control")
    
    # 3. GUIDEDモード待機
    wait_for_guided_mode(master)
    
    # 4. GPS Fix待機（EKF原点自動設定）
    start_position = wait_for_gps_fix(master)
    if not start_position:
        print("Failed to get GPS fix. Exiting.")
        sys.exit(1)
    
    # 5. アーム待機（ホームポジション自動設定）
    if not wait_for_arm(master):
        print("Failed to arm. Exiting.")
        sys.exit(1)
    
    # 6. 離陸
    takeoff(master, TAKEOFF_ALTITUDE)
    
    # 7. 精密制御開始
    precision_control(master, start_position)
    
    print("Program completed successfully")

if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    except Exception as e:
        print(f"\nError occurred: {e}")
        import traceback
        traceback.print_exc()
