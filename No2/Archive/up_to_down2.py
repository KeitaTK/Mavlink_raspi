#!/usr/bin/env python3
import sys, select, time, math, threading, termios, tty, csv, datetime, signal
from pathlib import Path
from pymavlink import mavutil
import pytz

# ───── ユーザー設定 ─────
STEP = 0.10
TAKEOFF_ALT = 0.50
SEND_HZ = 10
MASK = 0x09F8

# ───── 基準点設定 ─────
REF_LAT, REF_LON, REF_ALT = 36.0757800, 136.2132900, 0.0
TARGET_HEIGHT_ABOVE_TAKEOFF = 0.20
CSV_DIR = Path.home() / "LOGS_Pixhawk6c"
CSV_DIR.mkdir(exist_ok=True)

# ───── 状態変数 ─────
running = True
recording = False
target = {'x':0.0, 'y':0.0, 'z':0.0}
gps_now = {'x':0.0, 'y':0.0, 'z':0.0}
data_records = []
origin = None
io_lock = threading.Lock()
initial_target_set = False
initial_yaw, yaw_t_deg, yaw_acquired = None, 180.0, False

# ───── キー入力処理 ─────
def get_key():
    if select.select([sys.stdin], [], [], 0):
        return sys.stdin.read(1)
    return None

# ───── MAVLink接続と設定 ─────
def connect_mavlink():
    m = mavutil.mavlink_connection('/dev/ttyAMA0', baud=921600, rtscts=True)
    m.wait_heartbeat()
    print("✓ MAVLink接続完了")
    return m

def set_msg_rate(m):
    for mid in (24, 33, 30):
        m.mav.command_long_send(m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0, mid, 200000, 0,0,0,0,0)

def send_takeoff_command(m, alt):
    m.mav.command_long_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,0,0,0,0,0,0, alt)

def send_setpoint(m, lat_i, lon_i, alt, yaw_deg):
    m.mav.set_position_target_global_int_send(0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, MASK,
        lat_i, lon_i, alt, 0,0,0,0,0,0,
        math.radians(yaw_deg), 0)
    m.mav.command_long_send(m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0, yaw_deg, 20, 0,0,0,0,0)

# ───── ヨー角取得 ─────
def get_initial_yaw(m):
    print("現在のヨー角取得中...")
    for _ in range(10):
        att = m.recv_match(type='ATTITUDE', blocking=True, timeout=1)
        if att:
            yaw = (math.degrees(att.yaw) + 360) % 360
            print(f"✓ 基準ヨー角設定: {yaw:.1f}°")
            return yaw
    print("⚠ ヨー角取得失敗、デフォルト180°使用")
    return 180.0

# ───── 座標変換 ─────
def gps_to_local_xyz(lat, lon, alt):
    dx = (lon - REF_LON) * 111319.5 * math.cos(math.radians(REF_LAT))
    dy = (lat - REF_LAT) * 111319.5
    dz = alt - REF_ALT
    return dx, dy, dz

def local_xyz_to_gps(x, y, z):
    lat = REF_LAT + y / 111319.5
    lon = REF_LON + x / (111319.5 * math.cos(math.radians(REF_LAT)))
    alt = REF_ALT + z
    return lat, lon, alt

# ───── 状態監視スレッド ─────
def monitor_vehicle(m):
    global running, recording, gps_now, origin, initial_yaw, yaw_t_deg, yaw_acquired, initial_target_set
    guided, armed, takeoff_sent, takeoff_reached = False, False, False, False
    start_time = 0
    while running:
        hb = m.recv_match(type='HEARTBEAT', blocking=False, timeout=0.05)
        if hb:
            is_guided = hb.custom_mode == 4
            new_armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

            if is_guided and new_armed and not guided:
                print("✓ Guidedモード検出 → 離陸準備")
                start_time, guided = time.time(), True
                if not yaw_acquired:
                    initial_yaw = get_initial_yaw(m)
                    yaw_t_deg, yaw_acquired = initial_yaw, True

            if guided and not takeoff_sent and time.time() - start_time > 3:
                send_takeoff_command(m, TAKEOFF_ALT)
                print(f"✓ 離陸指令（{TAKEOFF_ALT:.1f}m）")
                recording, takeoff_sent = True, True

            if not is_guided:
                guided, takeoff_sent, recording, takeoff_reached, initial_target_set = False, False, False, False, False

            if not new_armed and recording:
                print("\n✓ ディスアーム検出 → 記録停止")
                running = recording = False

        pos = m.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if pos:
            lat, lon, alt = pos.lat / 1e7, pos.lon / 1e7, pos.relative_alt / 1000
            if origin is None:
                origin = pos
                print(f"✓ 原点設定 lat={lat}, lon={lon}")
            x, y, z = gps_to_local_xyz(lat, lon, alt)
            with io_lock:
                gps_now.update({'x': x, 'y': y, 'z': z})
            if takeoff_sent and not takeoff_reached and z >= TAKEOFF_ALT * 0.9:
                takeoff_reached = True
                with io_lock:
                    target['z'] = TAKEOFF_ALT
                    initial_target_set = True
                print(f"✓ 離陸高度到達: {target['z']:.2f}m")
        time.sleep(1 / SEND_HZ)

# ───── キーボード操作 ─────
def control_loop(m):
    global running, target, yaw_t_deg, initial_target_set
    fd, old = sys.stdin.fileno(), termios.tcgetattr(sys.stdin.fileno())
    tty.setcbreak(fd)
    try:
        print("\n" + "="*60)
        print("キーボード制御モード（u/m/h/l:XY, w/z:Z, a/d:Yaw, t:基準点）")
        print("="*60)
        while running:
            key, moved, echo = get_key(), False, None
            if key == 'q': running = False; break
            elif key in 'umhlwzadt':
                if key == 'u': target['y'] -= STEP; echo = "u South"
                elif key == 'm': target['y'] += STEP; echo = "m North"
                elif key == 'h': target['x'] += STEP; echo = "h East"
                elif key == 'l': target['x'] -= STEP; echo = "l West"
                elif key == 'w' and initial_target_set: target['z'] += STEP; echo = "w Up"
                elif key == 'z' and initial_target_set and target['z'] - STEP >= 0.05: target['z'] -= STEP; echo = "z Down"
                elif key == 'a': yaw_t_deg = (yaw_t_deg - 5) % 360; echo = "a Yaw-5"
                elif key == 'd': yaw_t_deg = (yaw_t_deg + 5) % 360; echo = "d Yaw+5"
                elif key == 't' and initial_target_set:
                    target['x'], target['y'], target['z'] = gps_to_local_xyz(36.0757693, 136.2132945, REF_ALT)
                    target['z'] = TARGET_HEIGHT_ABOVE_TAKEOFF
                    echo = f"t → X={target['x']:.2f} Y={target['y']:.2f} Z={target['z']:.2f}"
                if echo: moved = True; print(f"KEY: {echo}")
                if moved:
                    lat, lon, alt = local_xyz_to_gps(target['x'], target['y'], target['z'])
                    send_setpoint(m, int(lat*1e7), int(lon*1e7), alt, yaw_t_deg)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

# ───── データ記録 ─────
def record_data():
    while running:
        if recording:
            with io_lock:
                gps = gps_now.copy()
                tgt = target.copy()
            data_records.append([time.time(), gps['x'], gps['y'], gps['z'], tgt['x'], tgt['y'], tgt['z']])
        time.sleep(1 / SEND_HZ)

def save_csv():
    if not data_records:
        print("⚠ 記録なし")
        return
    now = datetime.datetime.now(pytz.timezone("Asia/Tokyo")).strftime("%Y%m%d_%H%M%S")
    path = CSV_DIR / f"{now}_2.csv"
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Time', 'GPS_X', 'GPS_Y', 'GPS_Z', 'Target_X', 'Target_Y', 'Target_Z'])
        writer.writerows(data_records)
    print(f"\n✓ CSV保存完了: {path} ({len(data_records)} 行)")

# ───── メイン関数 ─────
def main():
    global running
    signal.signal(signal.SIGINT, lambda s, f: setattr(sys.modules[__name__], "running", False))
    print("="*50)
    print("ArduPilot 精密制御 - 離陸高度自動設定")
    print("="*50)
    mav = connect_mavlink()
    set_msg_rate(mav)
    threading.Thread(target=monitor_vehicle, args=(mav,), daemon=True).start()
    threading.Thread(target=record_data, daemon=True).start()
    control_loop(mav)
    print("\n記録終了、CSV保存中...")
    save_csv()
    print("✓ プログラム終了")

if __name__ == "__main__":
    main()
