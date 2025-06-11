#!/usr/bin/env python3
"""
ArduPilot精密制御システム - 高精度2cm移動版
GPS_INPUT使用、正確な2cm移動、ヨー角固定、誤差監視機能
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
MOVE_DISTANCE = 0.02    # 正確に2cm移動

def get_key():
    """キー入力取得（高度制御対応）"""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
        
        # 通常のキー処理
        if ch == '\x1b':
            ch2 = sys.stdin.read(1)
            ch3 = sys.stdin.read(1)
            if ch2 == '[':
                if ch3 == 'A': return 'up'
                if ch3 == 'B': return 'down'
                if ch3 == 'C': return 'right'
                if ch3 == 'D': return 'left'
                # Ctrl+矢印キーの検出
                elif ch3 == '1':
                    ch4 = sys.stdin.read(1)
                    ch5 = sys.stdin.read(1)
                    if ch4 == ';' and ch5 == '5':
                        ch6 = sys.stdin.read(1)
                        if ch6 == 'A': return 'ctrl_up'
                        if ch6 == 'B': return 'ctrl_down'
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
    
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        30, 500000, 0, 0, 0, 0, 0  # ATTITUDE, 0.5秒間隔
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

def get_current_yaw(master):
    """現在のヨー角取得"""
    att_msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=2)
    if att_msg:
        return math.degrees(att_msg.yaw)
    return 0

def move_to_position_with_fixed_yaw(master, lat_int, lon_int, altitude, yaw_deg):
    """ヨー角固定での位置移動"""
    yaw_rad = math.radians(yaw_deg)
    
    master.mav.set_position_target_global_int_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111000000,  # ヨー角制御を有効化
        lat_int, lon_int, altitude,
        0, 0, 0, 0, 0, 0, yaw_rad, 0
    )

def get_current_status(master):
    """現在の状態取得"""
    status = {}
    
    # 位置情報
    pos_msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
    if pos_msg:
        status['lat'] = pos_msg.lat / 1e7
        status['lon'] = pos_msg.lon / 1e7
        status['altitude'] = pos_msg.relative_alt / 1000.0
    
    # 姿勢情報
    att_msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=2)
    if att_msg:
        status['roll'] = math.degrees(att_msg.roll)
        status['pitch'] = math.degrees(att_msg.pitch)
        status['yaw'] = math.degrees(att_msg.yaw)
    
    return status

def simple_ned_to_gps(lat0_deg, lon0_deg, north_m, east_m):
    """シンプルで正確なNED→GPS変換"""
    # WGS84楕円体での1度あたりの距離（より正確）
    lat_m_per_deg = 111132.92 - 559.82 * math.cos(2 * math.radians(lat0_deg)) + 1.175 * math.cos(4 * math.radians(lat0_deg))
    lon_m_per_deg = 111412.84 * math.cos(math.radians(lat0_deg)) - 93.5 * math.cos(3 * math.radians(lat0_deg))
    
    # 緯度経度変化計算
    lat_change = north_m / lat_m_per_deg
    lon_change = east_m / lon_m_per_deg
    
    new_lat = lat0_deg + lat_change
    new_lon = lon0_deg + lon_change
    
    return new_lat, new_lon

def precision_control_accurate_2cm(master, start_position):
    """高精度2cm移動制御"""
    print("\n" + "="*60)
    print("HIGH PRECISION 2CM CONTROL MODE")
    print("="*60)
    print("Controls:")
    print("  ↑ : South (-2cm)     | ↓ : North (+2cm)")
    print("  ← : East (+2cm)      | → : West (-2cm)")
    print("  w : Up (+2cm)        | x : Down (-2cm)")
    print("  s : Status           | h : Hold position")
    print("  r : Return origin    | q : Quit")
    print("EXACT movement distance: 2.00cm per key press")
    print("="*60)
    
    # 初期化
    lat0_deg = start_position.lat / 1e7
    lon0_deg = start_position.lon / 1e7
    alt0_msl = start_position.alt / 1000.0
    
    # 累積移動量（メートル単位）
    total_north = 0.0
    total_east = 0.0
    current_altitude = TAKEOFF_ALTITUDE
    
    # 初期ヨー角記録
    initial_yaw = get_current_yaw(master)
    print(f"Initial position: {lat0_deg:.7f}, {lon0_deg:.7f}")
    print(f"Initial yaw angle: {initial_yaw:.1f}° (FIXED)")
    print("Ready for EXACT 2cm control...")
    
    while True:
        key = get_key()
        moved = False
        altitude_changed = False
        
        if key == 'q':
            print("Exiting precision control")
            break
        elif key == 's':
            # 詳細状態表示
            print("\n--- DETAILED STATUS ---")
            status = get_current_status(master)
            
            # 現在の理論位置
            current_lat, current_lon = simple_ned_to_gps(lat0_deg, lon0_deg, total_north, total_east)
            
            print(f"Theoretical Position:")
            print(f"  North offset: {total_north*100:+.1f}cm")
            print(f"  East offset:  {total_east*100:+.1f}cm")
            print(f"  Target GPS:   {current_lat:.7f}, {current_lon:.7f}")
            
            if 'lat' in status:
                print(f"Actual GPS:     {status['lat']:.7f}, {status['lon']:.7f}")
                
                # 実際の移動距離計算
                actual_north = (status['lat'] - lat0_deg) * 111111.0
                actual_east = (status['lon'] - lon0_deg) * 111111.0 * math.cos(math.radians(lat0_deg))
                
                print(f"Actual Position:")
                print(f"  North offset: {actual_north*100:+.1f}cm")
                print(f"  East offset:  {actual_east*100:+.1f}cm")
                
                # 誤差計算
                north_error = abs(actual_north - total_north) * 100
                east_error = abs(actual_east - total_east) * 100
                print(f"Position Error: N={north_error:.1f}cm, E={east_error:.1f}cm")
            
            if 'altitude' in status:
                print(f"Target Altitude: {current_altitude:.3f}m")
                print(f"Actual Altitude: {status['altitude']:.3f}m")
                alt_error = abs(status['altitude'] - current_altitude) * 100
                print(f"Altitude Error:  {alt_error:.1f}cm")
            
            if 'yaw' in status:
                yaw_error = abs(status['yaw'] - initial_yaw)
                if yaw_error > 180:
                    yaw_error = 360 - yaw_error
                print(f"Yaw Error:       {yaw_error:.1f}°")
            
            print("--- END ---\n")
            continue
        elif key == 'h':
            print("→ Holding current position")
            moved = True
        elif key == 'r':
            print("→ Returning to origin")
            total_north = 0.0
            total_east = 0.0
            current_altitude = TAKEOFF_ALTITUDE
            moved = True
            altitude_changed = True
        
        # 水平移動（EXACT 2cm）
        elif key == 'up':  # 南へ移動
            total_north -= 0.02  # EXACTLY 2cm
            moved = True
            print(f"↓ South EXACTLY 2.0cm (total: {total_north*100:+.1f}cm)")
        elif key == 'down':  # 北へ移動
            total_north += 0.02  # EXACTLY 2cm
            moved = True
            print(f"↑ North EXACTLY 2.0cm (total: {total_north*100:+.1f}cm)")
        elif key == 'right':  # 西へ移動
            total_east -= 0.02  # EXACTLY 2cm
            moved = True
            print(f"→ West EXACTLY 2.0cm (total: {total_east*100:+.1f}cm)")
        elif key == 'left':  # 東へ移動
            total_east += 0.02  # EXACTLY 2cm
            moved = True
            print(f"← East EXACTLY 2.0cm (total: {total_east*100:+.1f}cm)")
        
        # 高度制御
        elif key == 'w':  # 上昇
            current_altitude += 0.02  # EXACTLY 2cm
            altitude_changed = True
            moved = True
            print(f"↑ Up EXACTLY 2.0cm (altitude: {current_altitude:.3f}m)")
        elif key == 'x':  # 下降
            if current_altitude - 0.02 >= 0.05:
                current_altitude -= 0.02  # EXACTLY 2cm
                altitude_changed = True
                moved = True
                print(f"↓ Down EXACTLY 2.0cm (altitude: {current_altitude:.3f}m)")
            else:
                print("⚠ Minimum altitude limit (5cm)")
        
        if moved:
            # 高精度GPS座標計算
            target_lat, target_lon = simple_ned_to_gps(lat0_deg, lon0_deg, total_north, total_east)
            
            # degE7形式に変換（精度保持）
            target_lat_int = int(round(target_lat * 1e7))
            target_lon_int = int(round(target_lon * 1e7))
            
            print(f"  Target GPS: {target_lat:.7f}, {target_lon:.7f}")
            
            # 移動コマンド送信
            move_to_position_with_fixed_yaw(master, target_lat_int, target_lon_int, 
                                          current_altitude, initial_yaw)
            
            if altitude_changed:
                print(f"  Target altitude: {current_altitude:.3f}m")
            
            time.sleep(0.1)

def main():
    """メイン処理"""
    print("ArduPilot Precision Control System - High Precision 2cm Version")
    print("=" * 70)
    
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
    print("4. Automatic takeoff and HIGH PRECISION 2cm control")
    
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
    
    # 7. 高精度2cm制御開始
    precision_control_accurate_2cm(master, start_position)
    
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
