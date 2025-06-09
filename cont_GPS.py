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
TAKEOFF_ALTITUDE = 0.5
MOVE_DISTANCE = 0.02

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

def request_all_messages(master):
    """必要なメッセージを全て要求"""
    print("Requesting all necessary messages...")
    
    # GPS関連メッセージ
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
    
    # システム状態メッセージ
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        0, 1000000, 0, 0, 0, 0, 0   # HEARTBEAT, 1秒間隔
    )
    
    # 互換性のためのストリーム要求
    master.mav.request_data_stream_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION, 2, 1
    )
    
    print("✓ Message requests sent")

def check_arm_status_detailed(master):
    """詳細なアーム状態確認"""
    print("\n=== DETAILED ARM STATUS CHECK ===")
    
    # HEARTBEATメッセージから状態確認
    heartbeat_msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
    if heartbeat_msg:
        # アーム状態の確認（複数の方法）
        armed_flag = bool(heartbeat_msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        custom_mode = heartbeat_msg.custom_mode
        system_status = heartbeat_msg.system_status
        
        print(f"Base Mode Armed Flag: {armed_flag}")
        print(f"master.motors_armed(): {master.motors_armed()}")
        print(f"Custom Mode: {custom_mode}")
        print(f"System Status: {system_status}")
        
        # システム状態の詳細
        status_names = {
            0: "BOOT", 1: "CALIBRATING", 2: "STANDBY", 3: "ACTIVE",
            4: "CRITICAL", 5: "EMERGENCY", 6: "POWEROFF", 7: "FLIGHT_TERMINATION"
        }
        status_name = status_names.get(system_status, f"Unknown({system_status})")
        print(f"System Status Name: {status_name}")
        
        return armed_flag
    else:
        print("✗ No HEARTBEAT message received")
        return False

def wait_for_arm_with_details(master, timeout=60):
    """詳細情報付きアーム待機"""
    print(f"\nWaiting for vehicle to be armed (timeout: {timeout}s)...")
    print("Please arm the vehicle using your transmitter:")
    print("  1. Throttle to minimum position")
    print("  2. Yaw stick to full right")  
    print("  3. Hold for 5 seconds")
    print("---")
    
    start_time = time.time()
    last_check_time = 0
    
    while time.time() - start_time < timeout:
        current_time = time.time()
        
        # 5秒ごとに詳細な状態確認
        if current_time - last_check_time >= 5:
            armed = check_arm_status_detailed(master)
            last_check_time = current_time
            
            if armed:
                print("✓ Vehicle is ARMED!")
                return True
            else:
                print("△ Vehicle still DISARMED, retrying...")
        
        # 高頻度でのアーム状態確認
        heartbeat_msg = master.recv_match(type='HEARTBEAT', blocking=False)
        if heartbeat_msg:
            armed = bool(heartbeat_msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            if armed:
                print("✓ Vehicle is ARMED!")
                return True
        
        time.sleep(0.5)
    
    print("✗ Arm timeout. Please check:")
    print("  - Pre-arm safety checks in Mission Planner")
    print("  - Transmitter stick positions")
    print("  - Battery voltage")
    return False

def wait_for_guided_mode(master):
    """GUIDEDモードになるまで待機"""
    print("Waiting for GUIDED mode...")
    while True:
        msg = master.recv_match(type='HEARTBEAT', blocking=True)
        if msg.custom_mode == 4:  # ArduCopterのGUIDEDモード
            print("Vehicle is in GUIDED mode.")
            break
        time.sleep(0.5)

def wait_for_gps_and_get_home(master):
    """GPS Fix待機とホームポジション取得"""
    print("Waiting for GPS fix...")
    
    for attempt in range(30):  # 30秒間試行
        gps_msg = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=2)
        
        if gps_msg and gps_msg.fix_type >= 3:  # 3D Fix以上
            fix_names = {3: "3D", 4: "DGPS", 5: "RTK Float", 6: "RTK Fixed"}
            print(f"✓ GPS: {fix_names.get(gps_msg.fix_type, gps_msg.fix_type)} | Sats: {gps_msg.satellites_visible}")
            
            # ホームポジション取得
            pos_msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=3)
            if pos_msg and pos_msg.lat != 0:
                lat_deg = pos_msg.lat / 1e7
                lon_deg = pos_msg.lon / 1e7
                alt_msl = pos_msg.alt / 1000.0
                print(f"✓ Home: Lat {lat_deg:.7f}°, Lon {lon_deg:.7f}°")
                return pos_msg
        
        time.sleep(1)
    
    return None

def arm_and_takeoff(master, altitude):
    """アーム確認と離陸"""
    # 改良されたアーム待機
    if not wait_for_arm_with_details(master):
        print("✗ Arming failed. Cannot proceed with takeoff.")
        return False

    print(f"\n🚀 Taking off to {altitude}m...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, altitude
    )

    print("Monitoring takeoff progress...")
    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=5)
        if msg:
            relative_alt_m = msg.relative_alt / 1000.0
            print(f"Current altitude: {relative_alt_m:.2f}m")
            if relative_alt_m >= altitude * 0.95:
                print("✓ Target altitude reached.")
                break
        else:
            print("⚠ No altitude data received")
        time.sleep(1)
    
    return True

def move_global_gps(master, lat_int, lon_int, altitude):
    """目標GPS座標へ移動"""
    print(f"Moving to Lat:{lat_int/1e7:.7f} Lon:{lon_int/1e7:.7f} Alt:{altitude:.2f}m")
    master.mav.set_position_target_global_int_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,
        lat_int, lon_int, altitude,
        0, 0, 0, 0, 0, 0, 0, 0
    )

# --- メイン処理 ---
if __name__ == "__main__":
    try:
        import pyned2lla
    except ImportError:
        print("Error: 'pyned2lla' library not found.")
        print("Please install it using: pip install pyned2lla")
        sys.exit(1)

    master = connect_to_vehicle(CONNECTION_PORT, BAUD_RATE)
    
    # 全メッセージ要求
    request_all_messages(master)
    time.sleep(2)  # メッセージ設定の待機
    
    print("\n--- 操作手順 ---")
    print("1. プロポで機体をアームしてください。")
    print("2. プロポでフライトモードを 'GUIDED' に切り替えてください。")
    
    wait_for_guided_mode(master)

    # GPS取得
    home_position = wait_for_gps_and_get_home(master)
    if not home_position:
        print("GPS acquisition failed. Exiting for safety.")
        sys.exit(1)
    
    lat0_deg = home_position.lat / 1e7
    lon0_deg = home_position.lon / 1e7
    alt0_msl = home_position.alt / 1000.0

    # 改良されたアーム確認と離陸
    if not arm_and_takeoff(master, TAKEOFF_ALTITUDE):
        print("Takeoff failed. Exiting.")
        sys.exit(1)

    # pyned2lla初期化
    wgs84 = pyned2lla.wgs84()
    lat0_rad = math.radians(lat0_deg)
    lon0_rad = math.radians(lon0_deg)
    target_north, target_east = 0, 0

    print("\n--- キーボード操作 ---")
    print("  Up Arrow:    北へ移動 | Down Arrow:  南へ移動")
    print("  Left Arrow:  西へ移動 | Right Arrow: 東へ移動")
    print("  q:           終了")
    print("----------------------")
    print("Ready for precision control...")

    while True:
        key = get_key()
        moved = False
        if key == 'q':
            print("Exiting script.")
            break
        elif key == 'up':
            target_north += MOVE_DISTANCE; moved = True
        elif key == 'down':
            target_north -= MOVE_DISTANCE; moved = True
        elif key == 'right':
            target_east += MOVE_DISTANCE; moved = True
        elif key == 'left':
            target_east -= MOVE_DISTANCE; moved = True

        if moved:
            (lat_rad, lon_rad, _) = pyned2lla.ned2lla(
                lat0_rad, lon0_rad, alt0_msl, 
                target_north, target_east, 0, wgs84
            )
            target_lat_int = int(math.degrees(lat_rad) * 1e7)
            target_lon_int = int(math.degrees(lon_rad) * 1e7)
            
            move_global_gps(master, target_lat_int, target_lon_int, TAKEOFF_ALTITUDE)
            time.sleep(0.1)

    print("Script finished.")
