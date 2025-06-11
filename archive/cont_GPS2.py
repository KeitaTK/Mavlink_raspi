#!/usr/bin/env python3
"""
ArduPilot精密制御システム - ノンブロッキング版
GPS_INPUT使用、リアルタイム状態監視、2cm精度制御
"""

import time
import sys
import select
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

class NonBlockingInput:
    """ノンブロッキング入力クラス"""
    
    def __init__(self):
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
    
    def get_key(self):
        """ノンブロッキングキー入力"""
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            ch = sys.stdin.read(1)
            if ch == '\x1b':  # エスケープシーケンス
                if select.select([sys.stdin], [], [], 0.1) == ([sys.stdin], [], []):
                    ch2 = sys.stdin.read(1)
                    if ch2 == '[':
                        if select.select([sys.stdin], [], [], 0.1) == ([sys.stdin], [], []):
                            ch3 = sys.stdin.read(1)
                            if ch3 == 'A': return 'up'
                            if ch3 == 'B': return 'down'
                            if ch3 == 'C': return 'right'
                            if ch3 == 'D': return 'left'
            return ch
        return None
    
    def __del__(self):
        """デストラクタ：端末設定を復元"""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

def connect_to_vehicle(port, baud):
    """機体に接続"""
    print(f"Connecting to vehicle on: {port} at {baud} baud")
    master = mavutil.mavlink_connection(port, baud=baud, wait_heartbeat=True)
    print(f"✓ Connected! System {master.target_system} Component {master.target_component}")
    return master

def setup_messages(master):
    """メッセージ要求"""
    print("Setting up messages...")
    
    messages = [
        (24, "GPS_RAW_INT"),
        (33, "GLOBAL_POSITION_INT"),
        (1, "SYS_STATUS"),
        (30, "ATTITUDE"),
        (193, "EKF_STATUS_REPORT")
    ]
    
    for msg_id, msg_name in messages:
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
            msg_id, 500000, 0, 0, 0, 0, 0  # 0.5秒間隔
        )
        time.sleep(0.1)
    
    print("✓ Message setup complete")

def wait_for_gps_fix(master):
    """GPS Fix待機"""
    print("Waiting for GPS fix...")
    
    for attempt in range(60):
        gps_msg = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=2)
        if gps_msg and gps_msg.fix_type >= 3:
            print(f"✓ GPS Fix: {gps_msg.fix_type}, Satellites: {gps_msg.satellites_visible}")
            
            pos_msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=3)
            if pos_msg and pos_msg.lat != 0:
                lat_deg = pos_msg.lat / 1e7
                lon_deg = pos_msg.lon / 1e7
                print(f"✓ Position: {lat_deg:.7f}, {lon_deg:.7f}")
                return pos_msg
        
        print(f"  Waiting... ({attempt + 1}/60)")
        time.sleep(1)
    
    return None

def wait_for_guided_mode(master):
    """GUIDEDモード待機"""
    print("Waiting for GUIDED mode...")
    while True:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg and msg.custom_mode == 4:
            print("✓ GUIDED mode active")
            break
        time.sleep(0.5)

def wait_for_arm(master):
    """アーム待機"""
    print("Waiting for vehicle to be armed...")
    print("Please arm using transmitter: Throttle down + Yaw right for 5 seconds")
    
    while True:
        heartbeat = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if heartbeat:
            armed = bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if armed:
                print("✓ Vehicle is ARMED!")
                return True
        time.sleep(0.5)

def takeoff(master, altitude):
    """離陸"""
    print(f"Taking off to {altitude}m...")
    
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, altitude
    )
    
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

def monitor_system_status(master):
    """システム状態監視（ノンブロッキング）"""
    status_info = {}
    
    # 高度情報
    pos_msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
    if pos_msg:
        status_info['altitude'] = pos_msg.relative_alt / 1000.0
    
    # 姿勢情報
    att_msg = master.recv_match(type='ATTITUDE', blocking=False)
    if att_msg:
        status_info['roll'] = math.degrees(att_msg.roll)
        status_info['pitch'] = math.degrees(att_msg.pitch)
        status_info['yaw'] = math.degrees(att_msg.yaw)
    
    # EKF状態
    ekf_msg = master.recv_match(type='EKF_STATUS_REPORT', blocking=False)
    if ekf_msg:
        status_info['ekf_flags'] = ekf_msg.flags
    
    return status_info

def precision_control_realtime(master, start_position):
    """リアルタイム精密制御ループ"""
    print("\n=== REAL-TIME PRECISION CONTROL MODE ===")
    print("Controls:")
    print("  ↑ : North (+2cm)  | ↓ : South (-2cm)")
    print("  ← : West (-2cm)   | → : East (+2cm)")
    print("  q : Quit          | s : Status")
    print("  h : Hold position | r : Reset to origin")
    print("Movement distance: 2cm per key press")
    print("System monitoring: Active")
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
    
    # ノンブロッキング入力初期化
    input_handler = NonBlockingInput()
    
    # 状態監視用変数
    last_status_time = time.time()
    last_command_time = time.time()
    
    try:
        while True:
            current_time = time.time()
            
            # === キー入力処理（ノンブロッキング） ===
            key = input_handler.get_key()
            moved = False
            
            if key == 'q':
                print("\nExiting precision control")
                break
            elif key == 's':
                # 状態表示
                status = monitor_system_status(master)
                print(f"\n--- STATUS ---")
                print(f"Position: N={total_north*100:+.0f}cm, E={total_east*100:+.0f}cm")
                if 'altitude' in status:
                    print(f"Altitude: {status['altitude']:.3f}m")
                if 'roll' in status:
                    print(f"Attitude: R={status['roll']:+.1f}°, P={status['pitch']:+.1f}°, Y={status['yaw']:+.1f}°")
                print("--- END ---\n")
            elif key == 'h':
                print("→ Holding current position")
                # 現在位置を維持（移動コマンド再送信）
                moved = True
            elif key == 'r':
                print("→ Resetting to origin")
                total_north = 0.0
                total_east = 0.0
                moved = True
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
            
            # === 移動コマンド送信 ===
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
                last_command_time = current_time
            
            # === 定期的な状態監視 ===
            if current_time - last_status_time >= 2.0:  # 2秒ごと
                status = monitor_system_status(master)
                
                # 高度チェック
                if 'altitude' in status:
                    alt_error = abs(status['altitude'] - TAKEOFF_ALTITUDE)
                    if alt_error > 0.05:  # 5cm以上の誤差
                        print(f"⚠ Altitude drift: {status['altitude']:.3f}m (target: {TAKEOFF_ALTITUDE:.3f}m)")
                
                # 姿勢チェック（振動検出）
                if 'roll' in status:
                    if abs(status['roll']) > 10 or abs(status['pitch']) > 10:
                        print(f"⚠ Large attitude: R={status['roll']:+.1f}°, P={status['pitch']:+.1f}°")
                
                last_status_time = current_time
            
            # === 位置維持コマンド（10秒ごと） ===
            if current_time - last_command_time >= 10.0:
                # 位置維持のため現在目標位置を再送信
                (target_lat_rad, target_lon_rad, _) = pyned2lla.ned2lla(
                    lat0_rad, lon0_rad, alt0_msl, 
                    total_north, total_east, 0, wgs84
                )
                target_lat_int = int(math.degrees(target_lat_rad) * 1e7)
                target_lon_int = int(math.degrees(target_lon_rad) * 1e7)
                move_to_position(master, target_lat_int, target_lon_int, TAKEOFF_ALTITUDE)
                last_command_time = current_time
                print("→ Position hold command sent")
            
            # CPU負荷軽減
            time.sleep(0.05)  # 20Hz更新
    
    finally:
        # 端末設定復元
        del input_handler

def main():
    """メイン処理"""
    print("ArduPilot Precision Control System - Real-time Version")
    print("=" * 60)
    
    try:
        import pyned2lla
    except ImportError:
        print("Error: pyned2lla not found. Install with: pip install pyned2lla")
        sys.exit(1)
    
    # 1. 接続
    master = connect_to_vehicle(CONNECTION_PORT, BAUD_RATE)
    
    # 2. メッセージ設定
    setup_messages(master)
    time.sleep(2)
    
    print("\n=== STARTUP SEQUENCE ===")
    print("1. Switch to GUIDED mode on transmitter")
    print("2. Wait for GPS fix and initialization")
    print("3. Arm the vehicle")
    print("4. Automatic takeoff and real-time precision control")
    
    # 3. GUIDEDモード待機
    wait_for_guided_mode(master)
    
    # 4. GPS Fix待機
    start_position = wait_for_gps_fix(master)
    if not start_position:
        print("Failed to get GPS fix. Exiting.")
        sys.exit(1)
    
    # 5. アーム待機
    if not wait_for_arm(master):
        print("Failed to arm. Exiting.")
        sys.exit(1)
    
    # 6. 離陸
    takeoff(master, TAKEOFF_ALTITUDE)
    
    # 7. リアルタイム精密制御開始
    precision_control_realtime(master, start_position)
    
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
