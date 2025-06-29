#!/usr/bin/env python3
"""
ArduPilot Precision Controller – v2
  • u/m/h/l : 10 cm  南 / 北 / 東 / 西
  • w / z   : 10 cm  上昇 / 下降
  • a / d   : Yaw –5° / +5°
  • 初期ヨー 180°（機首南向き）
  • 3 行表示 (KEY 行 + GPS/TARGET 行) を 5 Hz 更新
"""

import sys, select, time, math, threading, termios, tty
import pyned2lla
from pymavlink import mavutil

# ───── ユーザ設定 ─────
STEP        = 0.10           # 10 cm
TAKEOFF_ALT = 0.10           # 10 cm
GPS_HZ      = 5
MASK        = 0x09F8         # bit10=0(Yaw有効) bit11=1(YawRate無視)

# ───── 共有状態 ─────
gps_now   = {'lat':0,'lon':0,'alt':0}
target    = {'lat':0,'lon':0,'alt':0}
yaw_t_deg = 180.0            # 目標ヨー
yaw_now   = 0.0

io_lock = threading.Lock()
monitor = False

# ───── 非ブロッキングキー入力 ─────
def get_key():
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None

# ───── MAVLink接続 ─────
def connect():
    m = mavutil.mavlink_connection('/dev/ttyAMA0', 921600, wait_heartbeat=True, rtscts=True)
    print('✓ Heartbeat')
    return m

def set_msg_rate(m):
    for mid, us in [(24,200000),(33,200000),(30,200000)]:   # 5 Hz
        m.mav.command_long_send(m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0, mid, us, 0,0,0,0,0)

# ───── 表示スレッド ─────
def gps_monitor(m):
    global monitor, yaw_now
    while monitor:
        pos = m.recv_match(type='GLOBAL_POSITION_INT', blocking=False, timeout=0.05)
        att = m.recv_match(type='ATTITUDE',            blocking=False, timeout=0.05)
        if att:
            yaw_now = (math.degrees(att.yaw)+360)%360
        if pos:
            with io_lock:
                gps_now.update(lat=pos.lat/1e7, lon=pos.lon/1e7, alt=pos.relative_alt/1000)
                if target['lat']:
                    l1 = f"GPS:    {gps_now['lat']:.7f}, {gps_now['lon']:.7f}, Alt: {gps_now['alt']:.3f} m, Yaw: {yaw_now:6.1f}°"
                    l2 = f"TARGET: {target['lat']:.7f}, {target['lon']:.7f}, Alt: {target['alt']:.3f} m, Yaw: {yaw_t_deg:6.1f}°"
                    sys.stdout.write("\x1b[2K\r"+l1+"\n")
                    sys.stdout.write("\x1b[2K"+l2+"\r")
                    sys.stdout.flush()
        time.sleep(1/GPS_HZ)

# ───── 起動補助 ─────
def wait_guided(m):
    print('Waiting GUIDED…')
    while m.recv_match(type='HEARTBEAT',blocking=True).custom_mode!=4: pass
    print('✓ GUIDED')

def wait_fix(m):
    print('Waiting GPS fix…')
    while True:
        if (g:=m.recv_match(type='GPS_RAW_INT',blocking=True)) and g.fix_type>=3:
            p = m.recv_match(type='GLOBAL_POSITION_INT',blocking=True)
            print(f"✓ GPS OK {p.lat/1e7:.7f},{p.lon/1e7:.7f}")
            return p

def wait_arm(m):
    print('Arm vehicle…')
    while not (m.recv_match(type='HEARTBEAT',blocking=True)
                 .base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED): pass
    print('✓ Armed')

def takeoff(m, alt):
    print(f'Takeoff {alt} m')
    m.mav.command_long_send(m.target_system,m.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,0,0,0,0,0,0,0,alt)
    while True:
        if (m.recv_match(type='GLOBAL_POSITION_INT',blocking=True)
                .relative_alt/1000)>=alt*0.95:
            print('✓ Reached alt')
            return
        time.sleep(0.5)

# ───── コマンド送信 ─────
def send_setpoint(m, lat_i, lon_i, alt, yaw_deg):
    with io_lock:
        target.update(lat=lat_i/1e7, lon=lon_i/1e7, alt=alt)
    m.mav.set_position_target_global_int_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        MASK,
        lat_i, lon_i, alt,
        0,0,0, 0,0,0,
        math.radians(yaw_deg), 0)
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0, yaw_deg, 20, 0, 0, 0,0,0)

# ───── 操作ループ ─────
def control_loop(m, origin):
    global monitor, yaw_t_deg
    wgs = pyned2lla.wgs84()
    lat0, lon0 = origin.lat/1e7, origin.lon/1e7
    alt0 = origin.alt/1000
    lat0r, lon0r = map(math.radians,(lat0,lon0))
    n=e=0.0; alt=TAKEOFF_ALT

    fd = sys.stdin.fileno(); old=termios.tcgetattr(fd); tty.setraw(fd)
    monitor=True
    threading.Thread(target=gps_monitor,args=(m,),daemon=True).start()

    print("\n[u]南 [m]北 [h]東 [l]西 10 cm  [w/z]±10 cm  [a/d]Yaw±5°  [q]quit\n")
    send_setpoint(m,int(lat0*1e7),int(lon0*1e7),alt,yaw_t_deg)

    try:
        while True:
            k=get_key(); moved=False; echo=None
            if   k=='q': break
            elif k=='u': n-=STEP; moved=True; echo="u South"
            elif k=='m': n+=STEP; moved=True; echo="m North"
            elif k=='h': e+=STEP; moved=True; echo="h East "
            elif k=='l': e-=STEP; moved=True; echo="l West "
            elif k=='w': alt+=STEP; moved=True; echo="w Up  "
            elif k=='z':
                if alt-STEP>=0.05: alt-=STEP; moved=True; echo="z Down"
            elif k=='a': yaw_t_deg=(yaw_t_deg-5)%360; moved=True; echo="a Y-5"
            elif k=='d': yaw_t_deg=(yaw_t_deg+5)%360; moved=True; echo="d Y+5"

            if echo:
                sys.stdout.write("\x1b[2K\rKEY : "+echo+"\n"); sys.stdout.flush()

            if moved:
                latr,lonr,_=pyned2lla.ned2lla(lat0r,lon0r,alt0,n,e,0,wgs)
                send_setpoint(m,
                    int(math.degrees(latr)*1e7),
                    int(math.degrees(lonr)*1e7),
                    alt, yaw_t_deg)
            time.sleep(0.05)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        monitor=False
        print('\nExit.')

# ───── main ─────
def main():
    m=connect(); set_msg_rate(m); time.sleep(2)
    wait_guided(m); pos0=wait_fix(m); wait_arm(m); takeoff(m, TAKEOFF_ALT)
    control_loop(m,pos0)

if __name__=="__main__":
    main()
