#!/usr/bin/env python3
# ──────────────────────────────────────────────────────────────
# ArduPilot Precision Controller  ‑  simple 2-line GPS display +
#    key echo  +  arrow/WZ/AD yaw   (初期ヨー=南向き)
#
#   ↑ ↓ → ← : 2 cm  南 / 北 / 西 / 東
#   w / z   : 2 cm  上昇 / 下降
#   a / d   : 目標ヨー角 −5° / ＋5°   (0-360° wrap)
#   s       : 状態ダンプ      h : hold   r : origin   q : quit
#
#   キーを押すと   "KEY : xxx"  を 1 行出力し、
#   その下から 5 Hz で
#         GPS:    <現在値>
#         TARGET: <目標値>
#   の 2 行が更新され続ける。
# ──────────────────────────────────────────────────────────────

import sys, select, time, math, threading, termios, tty
import pyned2lla
from pymavlink import mavutil

# ───────── 設定値 ─────────
PORT        = '/dev/ttyAMA0'
BAUD        = 115200
TAKEOFF_ALT = 0.10          # 10 cm
STEP        = 0.02          # 2 cm
GPS_HZ      = 5
MASK        = 0x05F8        # 位置+Yaw 以外を無視

# ───────── グローバル共有 ─────────
gps_now   = {'lat': 0, 'lon': 0, 'alt': 0}
target    = {'lat': 0, 'lon': 0, 'alt': 0}
yaw_t_deg = 180.0           # ★ 初期ヨー角＝南
mon_flag  = False
io_lock   = threading.Lock()

# ──────────── キー取得（非ブロック）────────────
def get_key():
    if select.select([sys.stdin], [], [], 0)[0]:
        c = sys.stdin.read(1)
        if c == '\x1b':                               # ← 矢印開始
            if select.select([sys.stdin], [], [], 0)[0] and sys.stdin.read(1) == '[':
                k = sys.stdin.read(1)
                return {'A':'up', 'B':'down', 'C':'right', 'D':'left'}.get(k)
        return c
    return None

# ──────────── MAVLink 接続 ────────────
def connect():
    print(f'Connecting {PORT} …'); m = mavutil.mavlink_connection(PORT, baud=BAUD,
                                                                   wait_heartbeat=True)
    print('✓ Heartbeat'); return m

def req_rates(m):
    for mid, us in [(24,200000), (33,200000), (30,500000)]:
        m.mav.command_long_send(m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mid, us, 0,0,0,0,0)

# ──────────── 表示部 (GPS更新スレッド) ────────────
def gps_monitor(m):
    global mon_flag
    while mon_flag:
        pos = m.recv_match(type='GLOBAL_POSITION_INT', blocking=False, timeout=0.1)
        if pos:
            with io_lock:
                gps_now['lat'] = pos.lat/1e7
                gps_now['lon'] = pos.lon/1e7
                gps_now['alt'] = pos.relative_alt/1000.0

                if target['lat']:  # 目標が設定されたら 2 行表示
                    line1 = f"GPS:    {gps_now['lat']:.7f}, {gps_now['lon']:.7f}, Alt: {gps_now['alt']:.3f}m"
                    line2 = f"TARGET: {target['lat']:.7f}, {target['lon']:.7f}, Alt: {target['alt']:.3f}m"
                    # \x1b[2K = 行クリア, \r = 行頭復帰
                    sys.stdout.write("\x1b[2K\r" + line1 + "\n")
                    sys.stdout.write("\x1b[2K"   + line2 + "\r")
                    sys.stdout.flush()
        time.sleep(1/GPS_HZ)

# ──────────── ユーティリティ ────────────
def wait_fix(m):
    print('Waiting GPS-fix …')
    while True:
        g = m.recv_match(type='GPS_RAW_INT', blocking=True)
        if g and g.fix_type >= 3:
            pos = m.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if pos: return pos

def wait_mode(m):                     # GUIDED(4) 待機
    while m.recv_match(type='HEARTBEAT', blocking=True).custom_mode != 4: pass

def wait_arm(m):                      # ARM 待機
    while not (m.recv_match(type='HEARTBEAT', blocking=True)
                 .base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED): pass

def takeoff(m, alt):
    m.mav.command_long_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,0,0,0,0,0,0, alt)
    while True:
        if (m.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
                .relative_alt/1000.0) >= alt*0.95: break

def send_setpoint(m, lat_i, lon_i, alt, yaw_deg):
    with io_lock:
        target['lat'] = lat_i/1e7; target['lon'] = lon_i/1e7; target['alt'] = alt
    m.mav.set_position_target_global_int_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        MASK, lat_i, lon_i, alt,
        0,0,0, 0,0,0, math.radians(yaw_deg), 0)

# ──────────── メイン制御ループ ────────────
def control_loop(m, origin):
    global mon_flag, yaw_t_deg
    wgs = pyned2lla.wgs84()
    lat0 = origin.lat/1e7; lon0 = origin.lon/1e7
    lat0r, lon0r = map(math.radians, (lat0, lon0)); alt0 = origin.alt/1000.0

    n_off = e_off = 0.0; alt = TAKEOFF_ALT
    # 端末 RAW
    fd = sys.stdin.fileno(); old = termios.tcgetattr(fd); tty.setraw(fd)

    mon_flag = True
    threading.Thread(target=gps_monitor, args=(m,), daemon=True).start()
    print("\nControls: arrows/w z / a d | q quit\n")

    try:
        while True:
            k = get_key()
            moved = False
            if   k == 'q' : break
            elif k == 'up'   : n_off -= STEP; moved = True; echo="UP(S)"
            elif k == 'down' : n_off += STEP; moved = True; echo="DOWN(N)"
            elif k == 'right': e_off -= STEP; moved = True; echo="RIGHT(W)"
            elif k == 'left' : e_off += STEP; moved = True; echo="LEFT(E)"
            elif k == 'w'    : alt   += STEP; moved = True; echo="w +Alt"
            elif k == 'z'    : 
                if alt-STEP >= 0.05: alt -= STEP; moved = True; echo="z -Alt"
            elif k == 'a'    : yaw_t_deg = (yaw_t_deg-5)%360; moved = True; echo="a -Yaw"
            elif k == 'd'    : yaw_t_deg = (yaw_t_deg+5)%360; moved = True; echo="d +Yaw"
            else:             echo=None

            if echo:                   # キーを受け取ったら 1 行表示
                sys.stdout.write("\x1b[2K\rKEY : " + echo + "\n")  # 行クリア→表示→改行
                sys.stdout.flush()

            if moved:
                latr, lonr, _ = pyned2lla.ned2lla(lat0r, lon0r, alt0,
                                                  n_off, e_off, 0, wgs)
                send_setpoint(m,
                    int(math.degrees(latr)*1e7),
                    int(math.degrees(lonr)*1e7),
                    alt, yaw_t_deg)
            time.sleep(0.05)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        mon_flag = False
        print('\nExit.')

# ──────────── エントリポイント ────────────
def main():
    master = connect(); req_rates(master)
    wait_mode(master); pos0 = wait_fix(master)
    wait_arm(master);  takeoff(master, TAKEOFF_ALT)
    control_loop(master, pos0)

if __name__ == '__main__':
    main()
