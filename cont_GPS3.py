#!/usr/bin/env python3
# ──────────────────────────────────────────────────────────────
# ArduPilot Precision Controller
#   – 10 cm ステップ移動（矢印）
#   – w / z で ±10 cm 上下
#   – a / d で ヨー角 ±5°
#   – 初期ヨー 180°（機首が南）
#   – KEY 行 + GPS / TARGET 行（Yaw 付き）を 5 Hz で更新表示
# ──────────────────────────────────────────────────────────────

import sys, select, time, math, threading, termios, tty
import pyned2lla
from pymavlink import mavutil

# ───────── 設定値 ─────────
PORT        = '/dev/ttyAMA0'
BAUD        = 115200
STEP        = 0.10          # ← 10 cm
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

# ────────────────── キー読み取り
def get_key():
    if select.select([sys.stdin], [], [], 0)[0]:
        c = sys.stdin.read(1)
        if c == '\x1b':               # 矢印 = ESC [ A/B/C/D
            if select.select([sys.stdin], [], [], 0)[0] and sys.stdin.read(1) == '[':
                k = sys.stdin.read(1)
                return {'A':'up', 'B':'down', 'C':'right', 'D':'left'}.get(k)
        return c
    return None

# ────────────────── MAVLink
def connect():
    print(f'Connecting {PORT} …')
    m = mavutil.mavlink_connection(PORT, baud=BAUD, wait_heartbeat=True)
    print('✓ heartbeat')
    return m

def set_msg_rate(m):
    for mid, us in [(24,200000), (33,200000), (30,200000)]:  # GPS / GLOBAL_POS / ATT 5 Hz
        m.mav.command_long_send(m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0, mid, us, 0,0,0,0,0)

# ────────────────── 表示スレッド
def gps_monitor(m):
    global monitor, yaw_now
    while monitor:
        pos = m.recv_match(type='GLOBAL_POSITION_INT', blocking=False, timeout=0.05)
        att = m.recv_match(type='ATTITUDE', blocking=False, timeout=0.05)
        if att:
            yaw_now = (math.degrees(att.yaw) % 360)
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
    while m.recv_match(type='HEARTBEAT', blocking=True).custom_mode != 4: pass

def wait_fix(m):
    while True:
        g = m.recv_match(type='GPS_RAW_INT', blocking=True)
        if g and g.fix_type >= 3:
            p = m.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if p: return p

def wait_arm(m):
    while not (m.recv_match(type='HEARTBEAT', blocking=True)
                 .base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED): pass

def takeoff(m, alt):
    m.mav.command_long_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,
        0,0,0,0,0,0, alt)
    while True:
        if (m.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
                .relative_alt/1000.0) >= alt*0.95: break

# ────────────────── 送信
def send_setpoint(m, lat_i, lon_i, alt, yaw_deg):
    with io_lock:                         # 目標 GPS を確実に更新
        target['lat'] = lat_i/1e7
        target['lon'] = lon_i/1e7
        target['alt'] = alt
    m.mav.set_position_target_global_int_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        MASK,
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

    fd = sys.stdin.fileno(); old = termios.tcgetattr(fd); tty.setraw(fd)

    monitor = True
    threading.Thread(target=gps_monitor, args=(m,), daemon=True).start()
    print("\nControls: arrows 10 cm / w z ±10 cm / a d ±5° / q quit\n")

    try:
        while True:
            k = get_key()
            moved = False; echo = None
            if   k == 'q': break
            elif k == 'up'   : n -= STEP; moved=True; echo="↑ South 10 cm"
            elif k == 'down' : n += STEP; moved=True; echo="↓ North 10 cm"
            elif k == 'right': e -= STEP; moved=True; echo="→ West  10 cm"
            elif k == 'left' : e += STEP; moved=True; echo="← East  10 cm"
            elif k == 'w'    : alt += STEP; moved=True; echo="w  +Alt 10 cm"
            elif k == 'z'    : 
                if alt-STEP >= 0.05: alt -= STEP; moved=True; echo="z  -Alt 10 cm"
            elif k == 'a'    : yaw_t_deg = (yaw_t_deg-5)%360; moved=True; echo="a  Yaw -5°"
            elif k == 'd'    : yaw_t_deg = (yaw_t_deg+5)%360; moved=True; echo="d  Yaw +5°"

            if echo:
                sys.stdout.write("\x1b[2K\rKEY : " + echo + "\n")  # KEY 行
                sys.stdout.flush()

            if moved:
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
        print('\nExit.')

# ────────────────── main
def main():
    m = connect(); set_msg_rate(m)
    wait_guided(m)
    pos0 = wait_fix(m)
    wait_arm(m)
    takeoff(m, TAKEOFF_ALT)
    control_loop(m, pos0)

if __name__ == '__main__':
    main()
