#!/usr/bin/env python3
"""
ArduPilot精密制御システム - ヨー角制御無効版 + GPS誤差監視
GPS_INPUT使用、3次元精密制御、ヨー角制御なし、2cm精度制御、リアルタイム誤差表示
"""

import time
import sys
import select
import math
import threading
import pyned2lla
from pymavlink import mavutil

# --- 設定項目 ---
CONNECTION_PORT = '/dev/ttyAMA0'
BAUD_RATE = 115200
TAKEOFF_ALTITUDE = 0.1  # 10cm離陸
MOVE_DISTANCE = 0.02    # 2cm移動
GPS_UPDATE_RATE = 5     # Hz

# グローバル変数
current_target = {'lat': 0, 'lon': 0, 'alt': 0}
current_gps = {'lat': 0, 'lon': 0, 'alt': 0, 'timestamp': 0}
monitoring_active = False
lock = threading.Lock()

def get_non_blocking_key():
    """ノンブロッキングキー入力取得"""
    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        ch = sys.stdin.read(1)
        
        # エスケープシーケンス処理
        if ch == '\x1b':
            if select.select([sys.stdin], [], [], 0.1) == ([sys.stdin], [], []):
                ch2 = sys.stdin.read(1)
                if ch2 == '[':
                    if select.select([sys.stdin], [], [], 0.1) == ([sys.stdin], [], []):
                        ch3 = sys.stdin.read(1)
                        if ch3 == 'A': return 'up'
                        if ch3 == 'B': return 'down'
                        if ch3 == 'C': return 'right'
                        if ch3 == 'D': return 'left'
                        # Ctrl+矢印キー検出
                        elif ch3 == '1':
                            if select.select([sys.stdin], [], [], 0.1) == ([sys.stdin], [], []):
                                ch4 = sys.stdin.read(1)
                                if ch4 == ';':
                                    if select.select([sys.stdin], [], [], 0.1) == ([sys.stdin], [], []):
                                        ch5 = sys.stdin.read(1)
                                        if ch5 == '5':
                                            if select.select([sys.stdin], [], [], 0.1) == ([sys.stdin], [], []):
                                                ch6 = sys.stdin.read(1)
                                                if ch6 == 'A': return 'ctrl_up'
                                                if ch6 == 'B': return 'ctrl_down'
        return ch
    return None

def connect_to_vehicle(port, baud):
    """機体に接続"""
    print(f"Connecting to vehicle on: {port} at {baud} baud")
    master = mavutil.mavlink_connection(port, baud=baud, wait_heartbeat=True)
    print(f"✓ Connected! System {master.target_system} Component {master.target_component}")
    return master

def setup_minimal_messages(master):
    """最小限のメッセージ要求"""
    print("Setting up essential messages...")
    
    # GPS情報を高頻度で要求（200ms間隔 = 5Hz）
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        24, 200000, 0, 0, 0, 0, 0  # GPS_RAW_INT, 200ms間隔
    )
    
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        33, 200000, 0, 0, 0, 0, 0  # GLOBAL_POSITION_INT, 200ms間隔
    )
    
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
        30, 500000, 0, 0, 0, 0, 0  # ATTITUDE, 0.5秒間隔
    )
    
    print("✓ Message setup complete")

def gps_monitoring_thread(master):
    """GPS情報監視スレッド（5Hz）"""
    global current_gps, current_target, monitoring_active
    
    print("GPS monitoring thread started")
    
    while monitoring_active:
        try:
            # GPS情報取得
            pos_msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=False, timeout=0.1)
            
            if pos_msg:
                with lock:
                    current_gps['lat'] = pos_msg.lat / 1e7
                    current_gps['lon'] = pos_msg.lon / 1e7
                    current_gps['alt'] = pos_msg.relative_alt / 1000.0
                    current_gps['timestamp'] = time.time()
                
                # 誤差計算と表示
                if current_target['lat'] != 0:  # 目標が設定されている場合
                    lat_error = (current_gps['lat'] - current_target['lat']) * 111319.9  # 緯度差をメートルに変換
                    lon_error = (current_gps['lon'] - current_target['lon']) * 111319.9 * math.cos(math.radians(current_gps['lat']))
                    alt_error = current_gps['alt'] - current_target['alt']
                    
                    total_error = math.sqrt(lat_error**2 + lon_error**2 + alt_error**2)
                    
                    # 誤差表示（上書き形式）
                    error_line = (f"\r[GPS] Error: H={math.sqrt(lat_error**2 + lon_error**2)*100:.1f}cm, "
                                f"V={alt_error*100:+.1f}cm, Total={total_error*100:.1f}cm | "
                                f"GPS: {current_gps['lat']:.7f}, {current_gps['lon']:.7f}, {current_gps['alt']:.3f}m")
                    
                    print(error_line, end='', flush=True)
            
            time.sleep(1.0 / GPS_UPDATE_RATE)  # 5Hz
            
        except Exception as e:
            print(f"\nGPS monitoring error: {e}")
            time.sleep(0.5)

def wait_for_gps_fix(master):
    """GPS Fix待機（EKF原点自動設定）"""
    print("Waiting for GPS fix (EKF origin will be set automatically)...")
    
    for attempt in range(60):
        gps_msg = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=2)
        if gps_msg and gps_msg.fix_type >= 3:
            print(f"✓ GPS Fix: {gps_msg.fix_type}, Satellites: {gps_msg.satellites_visible}")
            print("✓ EKF origin automatically set")
            
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
        if msg.custom_mode == 4:
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
                print("✓ Home position automatically set")
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
    """指定GPS座標へ移動（ヨー角制御なし）"""
    global current_target
    
    # 目標位置更新
    with lock:
        current_target['lat'] = lat_int / 1e7
        current_target['lon'] = lon_int / 1e7
        current_target['alt'] = altitude
    
    # 目標位置表示
    print(f"\n[TARGET] Lat: {current_target['lat']:.7f}, Lon: {current_target['lon']:.7f}, Alt: {current_target['alt']:.3f}m")
    
    master.mav.set_position_target_global_int_send(
        0, master.target_system, master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,  # ヨー角制御無効
        lat_int, lon_int, altitude,
        0, 0, 0, 0, 0, 0, 0, 0
    )

def get_current_status(master):
    """現在の状態取得"""
    status = {}
    
    pos_msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=2)
    if pos_msg:
        status['lat'] = pos_msg.lat / 1e7
        status['lon'] = pos_msg.lon / 1e7
        status['altitude'] = pos_msg.relative_alt / 1000.0
    
    att_msg = master.recv_match(type='ATTITUDE', blocking=True, timeout=2)
    if att_msg:
        status['roll'] = math.degrees(att_msg.roll)
        status['pitch'] = math.degrees(att_msg.pitch)
        status['yaw'] = math.degrees(att_msg.yaw)
    
    return status

def precision_control(master, start_position):
    """精密制御ループ（ヨー角制御なし + リアルタイム誤差表示）"""
    global monitoring_active, current_target
    
    print("\n" + "="*80)
    print("PRECISION CONTROL MODE - NO YAW CONTROL + GPS ERROR MONITORING")
    print("="*80)
    print("Controls:")
    print("  ↑ : South (-2cm)     | ↓ : North (+2cm)")
    print("  ← : East (+2cm)      | → : West (-2cm)")
    print("  Ctrl+↑ : Up (+2cm)   | Ctrl+↓ : Down (-2cm)")
    print("  w : Up (+2cm)        | x : Down (-2cm)")
    print("  s : Status           | h : Hold position")
    print("  r : Return origin    | q : Quit")
    print("Movement: 2cm per key | Yaw: DISABLED | GPS Error: 5Hz update")
    print("="*80)
    
    # 初期化
    wgs84 = pyned2lla.wgs84()
    lat0_deg = start_position.lat / 1e7
    lon0_deg = start_position.lon / 1e7
    alt0_msl = start_position.alt / 1000.0
    lat0_rad = math.radians(lat0_deg)
    lon0_rad = math.radians(lon0_deg)
    
    total_north = 0.0
    total_east = 0.0
    current_altitude = TAKEOFF_ALTITUDE
    
    # 初期目標位置設定
    with lock:
        current_target['lat'] = lat0_deg
        current_target['lon'] = lon0_deg
        current_target['alt'] = current_altitude
    
    print(f"Initial position: {lat0_deg:.7f}, {lon0_deg:.7f}")
    
    # GPS監視スレッド開始
    monitoring_active = True
    gps_thread = threading.Thread(target=gps_monitoring_thread, args=(master,), daemon=True)
    gps_thread.start()
    
    print("Ready for precision control... (GPS error monitoring active)")
    
    # 端末設定（ノンブロッキング用）
    import termios, tty
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    tty.setraw(sys.stdin.fileno())
    
    try:
        while True:
            key = get_non_blocking_key()
            
            if key is None:
                time.sleep(0.05)  # CPU負荷軽減
                continue
            
            moved = False
            altitude_changed = False
            
            if key == 'q':
                print("\nExiting precision control")
                break
            elif key == 's':
                print(f"\n\n--- STATUS ---")
                print(f"Position: N={total_north*100:+.0f}cm, E={total_east*100:+.0f}cm")
                print(f"Target Altitude: {current_altitude:.3f}m")
                with lock:
                    print(f"Target GPS: {current_target['lat']:.7f}, {current_target['lon']:.7f}")
                    print(f"Current GPS: {current_gps['lat']:.7f}, {current_gps['lon']:.7f}")
                print("--- END ---\n")
                continue
            elif key == 'h':
                print("\n→ Holding current position and altitude")
                moved = True
            elif key == 'r':
                print("\n→ Returning to origin")
                total_north = 0.0
                total_east = 0.0
                current_altitude = TAKEOFF_ALTITUDE
                moved = True
                altitude_changed = True
            
            # 移動コマンド処理
            elif key == 'up':
                total_north -= MOVE_DISTANCE
                moved = True
                print(f"\n↓ South +{MOVE_DISTANCE*100:.0f}cm (total: {total_north*100:+.0f}cm)")
            elif key == 'down':
                total_north += MOVE_DISTANCE
                moved = True
                print(f"\n↑ North +{MOVE_DISTANCE*100:.0f}cm (total: {total_north*100:+.0f}cm)")
            elif key == 'right':
                total_east -= MOVE_DISTANCE
                moved = True
                print(f"\n→ West +{MOVE_DISTANCE*100:.0f}cm (total: {total_east*100:+.0f}cm)")
            elif key == 'left':
                total_east += MOVE_DISTANCE
                moved = True
                print(f"\n← East +{MOVE_DISTANCE*100:.0f}cm (total: {total_east*100:+.0f}cm)")
            elif key == 'ctrl_up' or key == 'w':
                current_altitude += MOVE_DISTANCE
                altitude_changed = True
                moved = True
                print(f"\n↑ Up +{MOVE_DISTANCE*100:.0f}cm (altitude: {current_altitude:.3f}m)")
            elif key == 'ctrl_down' or key == 'x':
                if current_altitude - MOVE_DISTANCE >= 0.05:
                    current_altitude -= MOVE_DISTANCE
                    altitude_changed = True
                    moved = True
                    print(f"\n↓ Down -{MOVE_DISTANCE*100:.0f}cm (altitude: {current_altitude:.3f}m)")
                else:
                    print("\n⚠ Minimum altitude limit (5cm)")
            
            if moved:
                # NED座標からGPS座標に変換
                (target_lat_rad, target_lon_rad, _) = pyned2lla.ned2lla(
                    lat0_rad, lon0_rad, alt0_msl, 
                    total_north, total_east, 0, wgs84
                )
                
                target_lat_int = int(math.degrees(target_lat_rad) * 1e7)
                target_lon_int = int(math.degrees(target_lon_rad) * 1e7)
                
                # 移動コマンド送信
                move_to_position(master, target_lat_int, target_lon_int, current_altitude)
                
                if altitude_changed:
                    print(f"  Target altitude set to: {current_altitude:.3f}m")
                
                time.sleep(0.1)
    
    finally:
        # 端末設定復元
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        monitoring_active = False

def main():
    """メイン処理"""
    global monitoring_active
    
    print("ArduPilot Precision Control System - Enhanced GPS Monitoring Version")
    print("=" * 80)
    
    try:
        import pyned2lla
    except ImportError:
        print("Error: pyned2lla not found. Install with: pip install pyned2lla")
        sys.exit(1)
    
    try:
        # 接続
        master = connect_to_vehicle(CONNECTION_PORT, BAUD_RATE)
        
        # メッセージ設定
        setup_minimal_messages(master)
        time.sleep(2)
        
        print("\n=== STARTUP SEQUENCE ===")
        print("1. Switch to GUIDED mode on transmitter")
        print("2. Wait for GPS fix and initialization")
        print("3. Arm the vehicle")
        print("4. Automatic takeoff and precision control with GPS monitoring")
        
        # 起動シーケンス
        wait_for_guided_mode(master)
        start_position = wait_for_gps_fix(master)
        if not start_position:
            print("Failed to get GPS fix. Exiting.")
            sys.exit(1)
        
        if not wait_for_arm(master):
            print("Failed to arm. Exiting.")
            sys.exit(1)
        
        takeoff(master, TAKEOFF_ALTITUDE)
        precision_control(master, start_position)
        
        print("Program completed successfully")
    
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
        monitoring_active = False
    except Exception as e:
        print(f"\nError occurred: {e}")
        monitoring_active = False
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main()
