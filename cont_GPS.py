import time
import sys
import tty
import termios
import math
import pyned2lla
from pymavlink import mavutil

# --- 設定項目 ---
# フライトコントローラーへの接続 (ラズパイのシリアルポート)
CONNECTION_PORT = '/dev/ttyAMA0'
BAUD_RATE = 115200

# 離陸する高度 (メートル)
TAKEOFF_ALTITUDE = 0.5

# 1回のキー入力での移動距離 (メートル)
MOVE_DISTANCE = 0.02

# --- 関数定義 (内容は前回と同じ) ---

def get_key():
    """ターミナルから1文字のキー入力を取得する関数 (Linux/macOS用)"""
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

def wait_for_guided_mode(master):
    """GUIDEDモードになるまで待機する"""
    print("Waiting for GUIDED mode...")
    while True:
        msg = master.recv_match(type='HEARTBEAT', blocking=True)
        if msg.custom_mode == 4: # ArduCopterのGUIDEDモードは 4
            print("Vehicle is in GUIDED mode.")
            break
        time.sleep(0.5)

def arm_and_takeoff(master, altitude):
    """機体をアーム(ここでは確認のみ)し、離陸させる"""
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
    """目標のGPS座標(GLOBAL_RELATIVE_ALT_INT)へ移動する"""
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

    print("Getting home position (GPS lock required)...")
    home_position = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=10)
    if home_position:
        lat0_deg = home_position.lat / 1e7
        lon0_deg = home_position.lon / 1e7
        alt0_msl = home_position.alt / 1000.0
        print(f"Home position acquired: Lat {lat0_deg:.7f}, Lon {lon0_deg:.7f}")
    else:
        print("Could not acquire home position. Check GPS lock and connection. Exiting.")
        sys.exit(1)

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
