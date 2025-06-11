#!/usr/bin/env python3
# ──────────────────────────────────────────────────────────────
# ArduPilot Precision Controller
#   – u/m/h/l キーで 10 cm ステップ移動（南/北/東/西）
#   – w/z で ±10 cm 上下
#   – a/d で ヨー角 ±5°
#   – 初期ヨー 180°（機首が南）
#   – KEY 行 + GPS / TARGET 行（Yaw 付き）を 5 Hz で更新表示
# ──────────────────────────────────────────────────────────────

import sys, select, time, math, threading, termios, tty
import pyned2lla
from pymavlink import mavutil

# ───────── 設定値 ─────────
PORT        = '/dev/ttyAMA0'
BAUD        = 115200
STEP        = 0.10          # 10 cm
TAKEOFF_ALT = 0.10          # 10 cm
GPS_HZ      = 5
MASK        = 0x05F8        # 位置 + ヨーのみ使用

# ───────── 共有変数 ─────────
gps_now   = {'lat': 0, 'lon': 0, 'alt': 0}
target    = {'lat': 0, 'lon': 0, 'alt': 0}
yaw_t_deg = 180.0           # ★南向き
yaw_now   = 0.0

io_lock   = threading.Lock()
monitor   = False

# ────────────────── キー読み取り（簡素化）
def get_key():
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None

# ────────────────── MAVLink
def connect():
    print(f'Connecting {PORT} …')
    m = mavutil.mavlink_connection(PORT, baud=BAUD, wait_heartbeat=True)
    print('✓ heartbeat')
    return m

def set_msg_rate(m):
    # GPS_RAW_INT, GLOBAL_POSITION_INT, ATTITUDE を 5Hz で要求
    for mid, us in [(24,200000), (33,200000), (30,200000)]:
        m.mav.command_long_send(m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0, mid, us, 0,0,0,0,0)

# ────────────────── 表示スレッド
def gps_monitor(m):
    global monitor, yaw_now
    while monitor:
        # GPS位置情報取得
        pos = m.recv_match(type='GLOBAL_POSITION_INT', blocking=False, timeout=0.05)
        # 姿勢情報取得
        att = m.recv_match(type='ATTITUDE', blocking=False, timeout=0.05)
        
        if att:
            yaw_now = (math.degrees(att.yaw) + 360) % 360  # 0-360度に正規化
            
        if pos:
            with io_lock:
                gps_now['lat'] = pos.lat/1e7
                gps_now['lon'] = pos.lon/1e7
                gps_now['alt'] = pos.relative_alt/1000.0
                
                if target['lat']:
                    l1 = (f"GPS:    {gps_now['lat']:.7f}, {gps_now['lon']:.7f}, "
                          f"Alt: {gps_now['alt']:.3f} m, Yaw: {yaw_now:6.1f}°")
                    l2 = (f"TARGET: {target['lat']:.7f}, {target['lon']:.7f}, "
                          f"Alt: {target['alt']:.3f} m, Yaw: {yaw_t_deg:6.1f}°")
                    # KEY 行の下に 2 行固定で上書き
                    sys.stdout.write("\x1b[2K\r" + l1 + "\n")
                    sys.stdout.write("\x1b[2K"   + l2 + "\r")
                    sys.stdout.flush()
        time.sleep(1/GPS_HZ)

# ────────────────── 小ユーティリティ
def wait_guided(m):
    print("Waiting for GUIDED mode...")
    while m.recv_match(type='HEARTBEAT', blocking=True).custom_mode != 4: 
        pass
    print("✓ GUIDED mode active")

def wait_fix(m):
    print("Waiting for GPS fix...")
    while True:
        g = m.recv_match(type='GPS_RAW_INT', blocking=True)
        if g and g.fix_type >= 3:
            p = m.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if p: 
                print(f"✓ GPS fix: {p.lat/1e7:.7f}, {p.lon/1e7:.7f}")
                return p

def wait_arm(m):
    print("Waiting for ARM...")
    while not (m.recv_match(type='HEARTBEAT', blocking=True)
                 .base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED): 
        pass
    print("✓ Vehicle ARMED")

def takeoff(m, alt):
    print(f"Taking off to {alt}m...")
    m.mav.command_long_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0,0,0,0,0,0, alt)
    while True:
        pos = m.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
        current_alt = pos.relative_alt/1000.0
        print(f"Altitude: {current_alt:.3f}m")
        if current_alt >= alt*0.95: 
            print("✓ Takeoff complete")
            break
        time.sleep(0.5)

# ────────────────── 送信
def send_setpoint(m, lat_i, lon_i, alt, yaw_deg):
    with io_lock:                         # 目標 GPS を確実に更新
        target['lat'] = lat_i/1e7
        target['lon'] = lon_i/1e7
        target['alt'] = alt
    
    # 位置とヨー角の両方を送信
    m.mav.set_position_target_global_int_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        MASK,  # 0x05F8 = 位置とヨー角のみ使用
        lat_i, lon_i, alt,
        0,0,0, 0,0,0,
        math.radians(yaw_deg), 0)

# ────────────────── 主ループ
def control_loop(m, origin):
    global monitor, yaw_t_deg
    wgs   = pyned2lla.wgs84()
    lat0  = origin.lat/1e7; lon0 = origin.lon/1e7
    alt0  = origin.alt/1000.0
    lat0r, lon0r = map(math.radians, (lat0, lon0))

    n = e = 0.0; alt = TAKEOFF_ALT

    # 端末をRAWモードに設定
    fd = sys.stdin.fileno(); old = termios.tcgetattr(fd); tty.setraw(fd)

    monitor = True
    threading.Thread(target=gps_monitor, args=(m,), daemon=True).start()
    
    print("\n" + "="*80)
    print("PRECISION CONTROL MODE")
    print("="*80)
    print("Controls:")
    print("  u : South 10 cm    | m : North 10 cm")
    print("  h : East  10 cm    | l : West  10 cm")
    print("  w : Up    10 cm    | z : Down  10 cm")
    print("  a : Yaw   -5°      | d : Yaw   +5°")
    print("  q : Quit")
    print("="*80)
    print()

    # 初期位置に移動（ヨー角設定）
    send_setpoint(m,
        int(lat0*1e7), int(lon0*1e7),
        alt, yaw_t_deg)

    try:
        while True:
            k = get_key()
            moved = False; echo = None
            
            if   k == 'q': break
            elif k == 'u': n -= STEP; moved=True; echo="u  South 10 cm"
            elif k == 'm': n += STEP; moved=True; echo="m  North 10 cm"
            elif k == 'h': e += STEP; moved=True; echo="h  East  10 cm"
            elif k == 'l': e -= STEP; moved=True; echo="l  West  10 cm"
            elif k == 'w': alt += STEP; moved=True; echo="w  Up    10 cm"
            elif k == 'z': 
                if alt-STEP >= 0.05: alt -= STEP; moved=True; echo="z  Down  10 cm"
                else: echo="z  (minimum altitude)"
            elif k == 'a': yaw_t_deg = (yaw_t_deg-5)%360; moved=True; echo="a  Yaw   -5°"
            elif k == 'd': yaw_t_deg = (yaw_t_deg+5)%360; moved=True; echo="d  Yaw   +5°"

            if echo:
                sys.stdout.write("\x1b[2K\rKEY : " + echo + "\n")  # KEY 行
                sys.stdout.flush()

            if moved:
                # NED座標からWGS84座標に変換
                latr, lonr, _ = pyned2lla.ned2lla(
                    lat0r, lon0r, alt0, n, e, 0, wgs)
                send_setpoint(m,
                    int(math.degrees(latr)*1e7),
                    int(math.degrees(lonr)*1e7),
                    alt, yaw_t_deg)
            time.sleep(0.05)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        monitor = False
        print('\nExiting precision control...')

# ────────────────── main
def main():
    print("ArduPilot Precision Control System - u/m/h/l Keys Version")
    print("=" * 80)
    
    try:
        m = connect()
        set_msg_rate(m)
        time.sleep(2)  # メッセージ設定待機
        
        wait_guided(m)
        pos0 = wait_fix(m)
        wait_arm(m)
        takeoff(m, TAKEOFF_ALT)
        control_loop(m, pos0)
        
        print("Program completed successfully")
        
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    except Exception as e:
        print(f"\nError occurred: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()
