# import time
# import sys
# import tty
# import termios
# import math
# import pyned2lla
# from pymavlink import mavutil

# # --- 設定項目 ---
# # フライトコントローラーへの接続 (ラズパイのシリアルポート)
# CONNECTION_PORT = '/dev/ttyAMA0'
# BAUD_RATE = 115200

# # 離陸する高度 (メートル)
# TAKEOFF_ALTITUDE = 0.5

# # 1回のキー入力での移動距離 (メートル)
# MOVE_DISTANCE = 0.02

# # --- 関数定義 (内容は前回と同じ) ---

# def get_key():
#     """ターミナルから1文字のキー入力を取得する関数 (Linux/macOS用)"""
#     fd = sys.stdin.fileno()
#     old_settings = termios.tcgetattr(fd)
#     try:
#         tty.setraw(sys.stdin.fileno())
#         ch = sys.stdin.read(1)
#         if ch == '\x1b':
#             ch2 = sys.stdin.read(1)
#             ch3 = sys.stdin.read(1)
#             if ch2 == '[':
#                 if ch3 == 'A': return 'up'
#                 if ch3 == 'B': return 'down'
#                 if ch3 == 'C': return 'right'
#                 if ch3 == 'D': return 'left'
#     finally:
#         termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
#     return ch

# def connect_to_vehicle(port, baud):
#     """機体に接続し、heartbeatを待つ"""
#     print(f"Connecting to vehicle on: {port} at {baud} baud")
#     master = mavutil.mavlink_connection(port, baud=baud, wait_heartbeat=True)
#     print(f"Heartbeat from system (system {master.target_system} component {master.target_component})")
#     return master

# def wait_for_guided_mode(master):
#     """GUIDEDモードになるまで待機する"""
#     print("Waiting for GUIDED mode...")
#     while True:
#         msg = master.recv_match(type='HEARTBEAT', blocking=True)
#         if msg.custom_mode == 4: # ArduCopterのGUIDEDモードは 4
#             print("Vehicle is in GUIDED mode.")
#             break
#         time.sleep(0.5)

# def arm_and_takeoff(master, altitude):
#     """機体をアーム(ここでは確認のみ)し、離陸させる"""
#     print("Waiting for vehicle to be armed...")
#     while not master.motors_armed():
#         time.sleep(1)
#     print("Vehicle is armed.")

#     print(f"Taking off to {altitude}m...")
#     master.mav.command_long_send(
#         master.target_system, master.target_component,
#         mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
#         0, 0, 0, 0, 0, 0, altitude
#     )

#     while True:
#         msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
#         relative_alt_m = msg.relative_alt / 1000.0
#         print(f"Current altitude: {relative_alt_m:.2f}m")
#         if relative_alt_m >= altitude * 0.95:
#             print("Target altitude reached.")
#             break
#         time.sleep(1)

# def move_global_gps(master, lat_int, lon_int, altitude):
#     """目標のGPS座標(GLOBAL_RELATIVE_ALT_INT)へ移動する"""
#     print(f"Moving to Lat:{lat_int/1e7:.7f} Lon:{lon_int/1e7:.7f} Alt:{altitude:.2f}m")
#     master.mav.set_position_target_global_int_send(
#         0, master.target_system, master.target_component,
#         mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
#         0b0000111111111000,
#         lat_int, lon_int, altitude,
#         0, 0, 0, 0, 0, 0, 0, 0
#     )

# # --- メイン処理 ---
# if __name__ == "__main__":
#     try:
#         import pyned2lla
#     except ImportError:
#         print("Error: 'pyned2lla' library not found.")
#         print("Please install it using: pip install pyned2lla")
#         sys.exit(1)

#     master = connect_to_vehicle(CONNECTION_PORT, BAUD_RATE)
    
#     print("\n--- 操作手順 ---")
#     print("1. プロポで機体をアームしてください。")
#     print("2. プロポでフライトモードを 'GUIDED' に切り替えてください。")
    
#     wait_for_guided_mode(master)

#     print("Getting home position (GPS lock required)...")
#     home_position = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=10)
#     if home_position:
#         lat0_deg = home_position.lat / 1e7
#         lon0_deg = home_position.lon / 1e7
#         alt0_msl = home_position.alt / 1000.0
#         print(f"Home position acquired: Lat {lat0_deg:.7f}, Lon {lon0_deg:.7f}")
#     else:
#         print("Could not acquire home position. Check GPS lock and connection. Exiting.")
#         sys.exit(1)

#     arm_and_takeoff(master, TAKEOFF_ALTITUDE)

#     wgs84 = pyned2lla.wgs84()
#     lat0_rad = math.radians(lat0_deg)
#     lon0_rad = math.radians(lon0_deg)
#     target_north, target_east = 0, 0

#     print("\n--- キーボード操作 ---")
#     print("  Up Arrow:    北へ移動 | Down Arrow:  南へ移動")
#     print("  Left Arrow:  西へ移動 | Right Arrow: 東へ移動")
#     print("  q:           終了")
#     print("----------------------")
#     print("Ready for keyboard commands...")

#     while True:
#         key = get_key()
#         moved = False
#         if key == 'q':
#             print("Exiting script.")
#             break
#         elif key == 'up':
#             target_north += MOVE_DISTANCE; moved = True
#         elif key == 'down':
#             target_north -= MOVE_DISTANCE; moved = True
#         elif key == 'right':
#             target_east += MOVE_DISTANCE; moved = True
#         elif key == 'left':
#             target_east -= MOVE_DISTANCE; moved = True

#         if moved:
#             (lat_rad, lon_rad, _) = pyned2lla.ned2lla(
#                 lat0_rad, lon0_rad, alt0_msl, 
#                 target_north, target_east, 0, wgs84
#             )
#             target_lat_int = int(math.degrees(lat_rad) * 1e7)
#             target_lon_int = int(math.degrees(lon_rad) * 1e7)
            
#             move_global_gps(master, target_lat_int, target_lon_int, TAKEOFF_ALTITUDE)
#             time.sleep(0.1)

#     print("Script finished.")


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

# GPS Fix状態の定義
GPS_FIX_TYPE = {
    0: "No GPS",
    1: "No Fix", 
    2: "2D Fix",
    3: "3D Fix",
    4: "DGPS",
    5: "RTK Float",
    6: "RTK Fixed"
}

# --- 関数定義 ---

def get_key():
    """ターミナルから1文字のキー入力を取得する関数"""
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
    """機体に接続し、heartbeatを待つ"""
    print(f"Connecting to vehicle on: {port} at {baud} baud")
    master = mavutil.mavlink_connection(port, baud=baud, wait_heartbeat=True)
    print(f"Heartbeat from system (system {master.target_system} component {master.target_component})")
    return master

def check_gps_status(master):
    """GPS状態の詳細確認（改良版）"""
    print("Checking GPS status...")
    
    for attempt in range(30):  # 最大30回（約30秒）試行
        # GPS生データの取得
        gps_raw_msg = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=2)
        
        if gps_raw_msg:
            fix_type = gps_raw_msg.fix_type
            satellites_visible = gps_raw_msg.satellites_visible
            hdop = gps_raw_msg.eph / 100.0  # cmからmに変換
            
            fix_status = GPS_FIX_TYPE.get(fix_type, f"Unknown({fix_type})")
            
            print(f"GPS Status: {fix_status} | Satellites: {satellites_visible} | HDOP: {hdop:.2f}")
            
            # 3D Fix以上で、衛星数が6個以上、HDOPが2.5以下なら合格
            if fix_type >= 3 and satellites_visible >= 6 and hdop <= 2.5:
                print("✓ GPS Fix OK! Acquiring position...")
                return True
            elif fix_type >= 3:
                print("△ GPS Fix achieved but quality may be poor. Continuing anyway...")
                return True
                
        else:
            print("× No GPS data received")
            
        time.sleep(1)
    
    print("✗ GPS Fix timeout. Please check GPS module and move to open sky area.")
    return False

def get_home_position_safe(master):
    """安全なホームポジション取得（改良版）"""
    print("Getting home position (GPS lock required)...")
    
    # まずGPS状態を確認
    if not check_gps_status(master):
        return None
    
    # GPS Fixが確認できたら位置情報を取得
    for attempt in range(10):  # 最大10回試行
        home_position = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=3)
        
        if home_position and home_position.lat != 0 and home_position.lon != 0:
            lat0_deg = home_position.lat / 1e7
            lon0_deg = home_position.lon / 1e7
            alt0_msl = home_position.alt / 1000.0  # mmからmに変換
            
            print(f"✓ Home position acquired:")
            print(f"  Latitude:  {lat0_deg:.7f}°")
            print(f"  Longitude: {lon0_deg:.7f}°") 
            print(f"  Altitude:  {alt0_msl:.2f}m (MSL)")
            
            return home_position
        
        print(f"Attempt {attempt + 1}/10: Invalid position data, retrying...")
        time.sleep(1)
    
    print("✗ Could not acquire valid home position.")
    return None

def wait_for_guided_mode(master):
    """GUIDEDモードになるまで待機する"""
    print("Waiting for GUIDED mode...")
    while True:
        msg = master.recv_match(type='HEARTBEAT', blocking=True)
        if msg.custom_mode == 4:  # ArduCopterのGUIDEDモードは 4
            print("Vehicle is in GUIDED mode.")
            break
        time.sleep(0.5)

def arm_and_takeoff(master, altitude):
    """機体をアーム(確認のみ)し、離陸させる"""
    print("Waiting for vehicle to be armed...")
    while not master.motors_armed():
        time.sleep(1)
    print("Vehicle is armed.")

    print(f"Taking off to {altitude}m...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0, 0, 0, 0, 0, 0, altitude
    )

    while True:
        msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        relative_alt_m = msg.relative_alt / 1000.0
        print(f"Current altitude: {relative_alt_m:.2f}m")
        if relative_alt_m >= altitude * 0.95:
            print("Target altitude reached.")
            break
        time.sleep(1)

def move_global_gps(master, lat_int, lon_int, altitude):
    """目標のGPS座標へ移動する"""
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
    
    print("\n--- 操作手順 ---")
    print("1. プロポで機体をアームしてください。")
    print("2. プロポでフライトモードを 'GUIDED' に切り替えてください。")
    
    wait_for_guided_mode(master)

    # 改良されたGPS位置取得
    home_position = get_home_position_safe(master)
    if not home_position:
        print("GPS acquisition failed. Exiting for safety.")
        sys.exit(1)
    
    lat0_deg = home_position.lat / 1e7
    lon0_deg = home_position.lon / 1e7
    alt0_msl = home_position.alt / 1000.0

    arm_and_takeoff(master, TAKEOFF_ALTITUDE)

    wgs84 = pyned2lla.wgs84()
    lat0_rad = math.radians(lat0_deg)
    lon0_rad = math.radians(lon0_deg)
    target_north, target_east = 0, 0

    print("\n--- キーボード操作 ---")
    print("  Up Arrow:    北へ移動 | Down Arrow:  南へ移動")
    print("  Left Arrow:  西へ移動 | Right Arrow: 東へ移動")
    print("  q:           終了")
    print("----------------------")
    print("Ready for keyboard commands...")

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
