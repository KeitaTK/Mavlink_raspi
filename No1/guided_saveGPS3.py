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
MESSAGE_INTERVAL_US = 500000  # 0.5秒間隔（メッセージ更新間隔）

class NonBlockingInput:
    """ノンブロッキング入力クラス"""
    def __init__(self):
        self.old_settings = termios.tcgetattr(sys.stdin)
        tty.setraw(sys.stdin.fileno())
    
    def get_key(self):
        """ノンブロッキングキー入力"""
        if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
            ch = sys.stdin.read(1)
            if ch == '\x1b':  # エスケープシーケンス（矢印キー）
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
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

def connect_to_vehicle(port, baud):
    print(f"Connecting to vehicle on: {port} at {baud} baud")
    master = mavutil.mavlink_connection(port, baud=baud, wait_heartbeat=True)
    print(f"✓ Connected! System {master.target_system} Component {master.target_component}")
    return master

def setup_messages(master):
    print("Setting up message intervals...")
    message_ids = [24, 33, 1, 30, 193]  # GPS_RAW_INT, GLOBAL_POSITION_INT, SYS_STATUS, ATTITUDE, EKF_STATUS_REPORT
    for msg_id in message_ids:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            msg_id,
            MESSAGE_INTERVAL_US,
            0, 0, 0, 0, 0
        )
        time.sleep(0.1)
    print("✓ Message setup complete")

def wait_for_gps_fix(master):
    print("Waiting for GPS fix...")
    for attempt in range(60):
        gps_msg = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=2)
        if gps_msg and gps_msg.fix_type >= 3:
            pos_msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=3)
            if pos_msg and pos_msg.lat != 0:
                lat_deg = pos_msg.lat / 1e7
                lon_deg = pos_msg.lon / 1e7
                print(f"✓ GPS Fix: lat={lat_deg:.7f}, lon={lon_deg:.7f}")
                return pos_msg
        print(f"  Waiting... ({attempt + 1}/60)")
        time.sleep(1)
    return None

def wait_for_guided_mode(master):
    print("Waiting for GUIDED mode...")
    while True:
        msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if msg and msg.custom_mode == 4:
            print("✓ GUIDED mode active")
            break
        time.sleep(0.5)

def wait_for_arm(master):
    print("Waiting for vehicle to be armed...")
    while True:
        heartbeat = master.recv_match(type='HEARTBEAT', blocking=True, timeout=1)
        if heartbeat:
            armed = bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if armed:
                print("✓ Vehicle is ARMED!")
                return True
        time.sleep(0.5)

def takeoff(master, altitude, initial_yaw_deg):
    print(f"Taking off to {altitude}m with yaw {initial_yaw_deg:.1f} deg...")
    
    # 離陸コマンド送信
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, altitude
    )
    
    # 離陸中もヨー角を維持するコマンドを送る（Yaw条件設定）
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0,
        initial_yaw_deg,
        0, 0, 1, 0, 0, 0
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

def move_to_position(master, lat_int, lon_int, altitude, yaw_deg):
    # 位置とヨー角をセット
    master.mav.set_position_target_global_int_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,  # 位置 + Yaw指定ビットセット
        lat_int, lon_int, altitude,
        0, 0, 0,  # velocity (unused)
        0, 0, 0,  # acceleration (unused)
        math.radians(yaw_deg),
        0  # Yaw rate 0で角速度指定なし
    )

def monitor_system_status(master):
    status_info = {}
    pos_msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
    if pos_msg:
        status_info['altitude'] = pos_msg.relative_alt / 1000.0
    att_msg = master.recv_match(type='ATTITUDE', blocking=False)
    if att_msg:
        status_info['roll'] = math.degrees(att_msg.roll)
        status_info['pitch'] = math.degrees(att_msg.pitch)
        status_info['yaw'] = (math.degrees(att_msg.yaw) + 360) % 360
    ekf_msg = master.recv_match(type='EKF_STATUS_REPORT', blocking=False)
    if ekf_msg:
        status_info['ekf_flags'] = ekf_msg.flags
    return status_info

def precision_control_realtime(master, start_position):
    print("\n=== REAL-TIME PRECISION CONTROL MODE ===")
    print("Controls:")
    print("  ↑ : North (+2cm)  | ↓ : South (-2cm)")
    print("  ← : West (-2cm)   | → : East (+2cm)")
    print("  a/d : Yaw -/+ 5°")
    print("  q : Quit")
    print("Movement distance: 2cm per key press")
    print("Ready for control...")

    wgs84 = pyned2lla.wgs84()
    lat0_deg = start_position.lat / 1e7
    lon0_deg = start_position.lon / 1e7
    alt0_msl = start_position.alt / 1000.0
    lat0_rad = math.radians(lat0_deg)
    lon0_rad = math.radians(lon0_deg)

    total_north = 0.0
    total_east = 0.0
    yaw_deg = None

    # 初期ヨー角を取得し保持
    status = monitor_system_status(master)
    if 'yaw' in status:
        yaw_deg = status['yaw']
    else:
        yaw_deg = 180.0  # デフォルト南向き
    print(f"Initial yaw: {yaw_deg:.1f} deg")

    input_handler = NonBlockingInput()
    last_command_time = time.time()

    try:
        while True:
            key = input_handler.get_key()
            moved = False

            if key == 'q':
                print("\nExiting precision control")
                break
            elif key == 'up':
                total_north += MOVE_DISTANCE
                moved = True
                print(f"→ North +{MOVE_DISTANCE*100:.0f}cm (total N={total_north*100:+.0f}cm)")
            elif key == 'down':
                total_north -= MOVE_DISTANCE
                moved = True
                print(f"→ South +{MOVE_DISTANCE*100:.0f}cm (total N={total_north*100:+.0f}cm)")
            elif key == 'right':
                total_east += MOVE_DISTANCE
                moved = True
                print(f"→ East +{MOVE_DISTANCE*100:.0f}cm (total E={total_east*100:+.0f}cm)")
            elif key == 'left':
                total_east -= MOVE_DISTANCE
                moved = True
                print(f"→ West +{MOVE_DISTANCE*100:.0f}cm (total E={total_east*100:+.0f}cm)")
            elif key == 'a':
                yaw_deg = (yaw_deg - 5) % 360
                moved = True
                print(f"→ Yaw -5° → {yaw_deg:.1f}°")
            elif key == 'd':
                yaw_deg = (yaw_deg + 5) % 360
                moved = True
                print(f"→ Yaw +5° → {yaw_deg:.1f}°")

            if moved:
                target_lat_rad, target_lon_rad, _ = pyned2lla.ned2lla(
                    lat0_rad, lon0_rad, alt0_msl,
                    total_north, total_east, 0, wgs84
                )
                target_lat_int = int(math.degrees(target_lat_rad) * 1e7)
                target_lon_int = int(math.degrees(target_lon_rad) * 1e7)
                move_to_position(master, target_lat_int, target_lon_int, TAKEOFF_ALTITUDE, yaw_deg)
                last_command_time = time.time()

            # 10秒ごとに目標位置とヨー角を再送信して位置保持
            if time.time() - last_command_time >= 10.0:
                target_lat_rad, target_lon_rad, _ = pyned2lla.ned2lla(
                    lat0_rad, lon0_rad, alt0_msl,
                    total_north, total_east, 0, wgs84
                )
                target_lat_int = int(math.degrees(target_lat_rad) * 1e7)
                target_lon_int = int(math.degrees(target_lon_rad) * 1e7)
                move_to_position(master, target_lat_int, target_lon_int, TAKEOFF_ALTITUDE, yaw_deg)
                last_command_time = time.time()
                print("→ Position hold command sent")

            time.sleep(0.05)

    finally:
        del input_handler

def main():
    print("ArduPilot Precision Control System - Real-time Version")
    print("=" * 60)

    master = connect_to_vehicle(CONNECTION_PORT, BAUD_RATE)
    setup_messages(master)
    time.sleep(2)

    print("\n=== STARTUP SEQUENCE ===")
    print("1. Switch to GUIDED mode on transmitter")
    print("2. Wait for GPS fix and initialization")
    print("3. Arm the vehicle")
    print("4. Automatic takeoff and real-time precision control")

    wait_for_guided_mode(master)
    start_position = wait_for_gps_fix(master)
    if not start_position:
        print("Failed to get GPS fix. Exiting.")
        sys.exit(1)

    if not wait_for_arm(master):
        print("Failed to arm. Exiting.")
        sys.exit(1)

    # ここで離陸時のヨー角を取得
    status = monitor_system_status(master)
    if 'yaw' in status:
        initial_yaw_deg = status['yaw']
    else:
        initial_yaw_deg = 180.0
    print(f"Initial yaw angle: {initial_yaw_deg:.1f} deg")

    takeoff(master, TAKEOFF_ALTITUDE, initial_yaw_deg)

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
