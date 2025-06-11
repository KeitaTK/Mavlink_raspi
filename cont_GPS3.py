#!/usr/bin/env python3
"""
ArduPilot 精密制御システム  ─  シンプル 2 行表示 + WASD/矢印 + ヨー制御
  • 矢印↑↓→← : 2 cm 単位で  南 / 北 / 西 / 東 へ水平移動
  • w / z     : 2 cm 単位で  上昇 / 下降
  • a / d     : Yaw 目標角を −5° / ＋5°（0-360°ラップ）
  • s         : 状態表示　h : ホールド　r : 原点へ戻る　q : 終了
"""

import sys, select, time, math, threading, termios, tty
import pyned2lla
from pymavlink import mavutil

# ─────────────────────────────────────  ユーザー設定
PORT              = '/dev/ttyAMA0'
BAUD              = 115200
TAKEOFF_ALT       = 0.10       # 10 cm
STEP              = 0.02       # 2 cm
GPS_HZ            = 5
TYPE_MASK_POS_YAW = 0x05F8     # 速度・加速度・YawRate 無視、Yaw 有効
# ─────────────────────────────────────────────────────

current_target = {'lat': 0, 'lon': 0, 'alt': 0}
current_gps    = {'lat': 0, 'lon': 0, 'alt': 0}
yaw_target_deg = 0.0

lock = threading.Lock()
monitoring = False

# ─────────────────────────────  キーボード入力（ノンブロッキング）
def get_key():
    if select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], []):
        ch = sys.stdin.read(1)
        if ch == '\x1b':                                  # ─ 矢印
            if select.select([sys.stdin], [], [], 0.05)[0]:
                if sys.stdin.read(1) == '[':
                    k = sys.stdin.read(1)
                    return {'A': 'up', 'B': 'down',
                            'C': 'right', 'D': 'left'}.get(k)
        return ch                                         # 単独キー
    return None

# ─────────────────────────────  MAVLink 接続 & 初期化
def connect():
    print(f'Connecting {PORT} …')
    m = mavutil.mavlink_connection(PORT, baud=BAUD, wait_heartbeat=True)
    print('✓ heartbeat OK')
    return m

def set_msg_rate(m):
    for mid, us in [(24,200000), (33,200000), (30,500000)]:     # GPS / GLOBAL_POS / ATT
        m.mav.command_long_send(m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0, mid, us, 0,0,0,0,0)

# ─────────────────────────────  GPS モニタ（２ 行表示のみ）
def gps_thread(master):
    global monitoring
    while monitoring:
        msg = master.recv_match(type='GLOBAL_POSITION_INT',
                                blocking=False, timeout=0.1)
        if msg:
            with lock:
                current_gps['lat'] = msg.lat / 1e7
                current_gps['lon'] = msg.lon / 1e7
                current_gps['alt'] = msg.relative_alt / 1000.0
            if current_target['lat']:
                l1 = f"GPS:    {current_gps['lat']:.7f}, {current_gps['lon']:.7f}, Alt: {current_gps['alt']:.3f}m"
                l2 = f"TARGET: {current_target['lat']:.7f}, {current_target['lon']:.7f}, Alt: {current_target['alt']:.3f}m"
                print(f"\r{l1}\n{l2}", end='\r', flush=True)
        time.sleep(1/GPS_HZ)

# ─────────────────────────────  基本ユーティリティ
def wait_fix(m):
    print('Waiting GPS-fix …')
    while True:
        g = m.recv_match(type='GPS_RAW_INT', blocking=True)
        if g and g.fix_type >= 3:
            pos = m.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            if pos: return pos

def wait_mode_guided(m):
    while True:
        if m.recv_match(type='HEARTBEAT', blocking=True).custom_mode == 4:
            return

def wait_arm(m):
    while True:
        hb = m.recv_match(type='HEARTBEAT', blocking=True)
        if hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
            return

def takeoff(m, alt):
    m.mav.command_long_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,0,0,0,0,0,0, alt)
    while True:
        if (m.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
                .relative_alt / 1000.0) >= alt*0.95:
            return

# ─────────────────────────────  位置・ヨー目標送信
def send_setpoint(m, lat_i, lon_i, alt, yaw_deg):
    with lock:
        current_target['lat'] = lat_i/1e7
        current_target['lon'] = lon_i/1e7
        current_target['alt'] = alt
    m.mav.set_position_target_global_int_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        TYPE_MASK_POS_YAW,
        lat_i, lon_i, alt,
        0,0,0, 0,0,0,
        math.radians(yaw_deg), 0)

# ─────────────────────────────  主制御ループ
def precision_control(m, origin):
    global monitoring, yaw_target_deg
    wgs84   = pyned2lla.wgs84()
    lat0_d  = origin.lat/1e7; lon0_d = origin.lon/1e7
    lat0_r  = math.radians(lat0_d); lon0_r = math.radians(lon0_d)
    alt0    = origin.alt/1000.0

    n_tot = e_tot = 0.0
    alt   = TAKEOFF_ALT

    # コンソール準備
    fd = sys.stdin.fileno(); old = termios.tcgetattr(fd); tty.setraw(fd)

    monitoring = True
    threading.Thread(target=gps_thread, args=(m,), daemon=True).start()
    print("\nControls: ↑↓→← move 2 cm | w/z ±2 cm alt | a/d ±5° yaw | q quit\n")

    try:
        while True:
            k = get_key()
            moved = False
            if k == 'q': break
            elif k == 'up'   : n_tot -= STEP; moved = True        # 南
            elif k == 'down' : n_tot += STEP; moved = True        # 北
            elif k == 'right': e_tot -= STEP; moved = True        # 西
            elif k == 'left' : e_tot += STEP; moved = True        # 東
            elif k == 'w'    : alt   += STEP; moved = True
            elif k == 'z'    : 
                if alt-STEP >= 0.05: alt -= STEP; moved = True
            elif k == 'a'    : yaw_target_deg = (yaw_target_deg-5)%360; moved = True
            elif k == 'd'    : yaw_target_deg = (yaw_target_deg+5)%360; moved = True

            if moved:
                lat_r, lon_r, _ = pyned2lla.ned2lla(
                    lat0_r, lon0_r, alt0, n_tot, e_tot, 0, wgs84)
                send_setpoint(m,
                    int(math.degrees(lat_r)*1e7),
                    int(math.degrees(lon_r)*1e7),
                    alt, yaw_target_deg)
            time.sleep(0.05)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        monitoring = False
        print('\nExit.')

# ─────────────────────────────  エントリポイント
def main():
    master = connect(); set_msg_rate(master)
    wait_mode_guided(master); pos0 = wait_fix(master)
    wait_arm(master); takeoff(master, TAKEOFF_ALT)
    precision_control(master, pos0)

if __name__ == '__main__':
    main()
