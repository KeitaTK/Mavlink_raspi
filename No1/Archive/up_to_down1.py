#!/usr/bin/env python3
import sys, select, time, math, threading, termios, tty, csv, datetime, signal
from pathlib import Path
from pymavlink import mavutil
import pytz
import pyned2lla

# ───── ユーザー設定 ─────
STEP = 0.10          # 10cm移動
TAKEOFF_ALT = 0.50   # 初期離陸高度（m）
SEND_HZ = 10
MASK = 0x09F8        # bit10=0(Yaw有効) bit11=1(YawRate無視)

# ───── 基準点設定 ─────
REF_LAT = 36.0757800               # 基準GPS緯度
REF_LON = 136.2132900              # 基準GPS経度
REF_ALT = 0.000                    # 基準GPS高度
TARGET_HEIGHT_ABOVE_TAKEOFF = 1.10 # 離陸地点から70cm上空

CSV_DIR = Path.home() / "LOGS_Pixhawk6c"
CSV_DIR.mkdir(exist_ok=True)

# ───── 状態変数 ─────
running = True
recording = False
target = {'x':0.0, 'y':0.0, 'z':0.0}  # ローカル座標系で記録
gps_now = {'x':0.0, 'y':0.0, 'z':0.0}
data_records = []
origin = None
io_lock = threading.Lock()
initial_target_set = False  # 初期目標高度設定完了フラグ

# ───── ヨー角制御関連 ─────
initial_yaw = None
yaw_t_deg = 180.0
yaw_acquired = False

# ───── 非ブロッキングキー入力 ─────
def get_key():
    if select.select([sys.stdin], [], [], 0):
        return sys.stdin.read(1)
    return None

# ───── MAV接続 & 設定 ─────
def connect_mavlink():
    m = mavutil.mavlink_connection('/dev/ttyAMA0', baud=1000000, rtscts=True)
    m.wait_heartbeat()
    print("✓ MAVLink接続完了")
    return m

def set_msg_rate(m):
    for mid, us in [(24,200000),(33,200000),(30,200000)]:
        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0, mid, us, 0,0,0,0,0
        )

def send_takeoff_command(mav, alt):
    mav.mav.command_long_send(
        mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,0,0,0,0,0,0, alt
    )

def send_setpoint(m, lat_i, lon_i, alt, yaw_deg):
    m.mav.set_position_target_global_int_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        MASK,
        lat_i, lon_i, alt,
        0,0,0,0,0,0,
        math.radians(yaw_deg), 0
    )
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0, yaw_deg, 20, 0,0,0,0,0
    )

# ───── 離陸時ヨー角取得 ─────
def get_initial_yaw(mav):
    print("現在のヨー角取得中...")
    for _ in range(10):
        att = mav.recv_match(type='ATTITUDE', blocking=True, timeout=1)
        if att:
            yaw = (math.degrees(att.yaw) + 360) % 360
            print(f"✓ 基準ヨー角設定: {yaw:.1f}°")
            return yaw
        time.sleep(0.1)
    print("⚠ ヨー角取得失敗、デフォルト値180°を使用")
    return 180.0

# ───── GPS → ローカルXYZ変換 ─────
def gps_to_local_xyz(lat, lon, alt):
    if origin is None:
        return 0.0, 0.0, 0.0
    lat0 = origin.lat / 1e7
    lon0 = origin.lon / 1e7
    alt0 = origin.relative_alt / 1000
    x = (lon - lon0) * 111319.5 * math.cos(math.radians(lat0))
    y = (lat - lat0) * 111319.5
    z = alt - alt0
    return x, y, z

# ───── ローカルXYZ → 緯度経度変換 ─────
def local_xyz_to_gps(x, y, z):
    if origin is None:
        return 0.0, 0.0, 0.0
    lat0 = origin.lat / 1e7
    lon0 = origin.lon / 1e7
    alt0 = origin.relative_alt / 1000
    lat = lat0 + y / 111319.5
    lon = lon0 + x / (111319.5 * math.cos(math.radians(lat0)))
    alt = alt0 + z
    return lat, lon, alt

# ───── GPS更新 & ディスアーム監視 ─────
def monitor_vehicle(mav):
    global running, recording, gps_now, origin
    global initial_yaw, yaw_t_deg, yaw_acquired, initial_target_set
    guided_active = False
    armed = False
    takeoff_sent = False
    takeoff_reached = False
    start_time = 0

    while running:
        hb = mav.recv_match(type='HEARTBEAT', blocking=False, timeout=0.05)
        if hb:
            mode = hb.custom_mode
            new_armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

            if mode == 4 and new_armed and not guided_active:
                print("✓ Guidedモード検出 → 離陸準備")
                start_time = time.time()
                guided_active = True

                if not yaw_acquired:
                    initial_yaw = get_initial_yaw(mav)
                    yaw_t_deg = initial_yaw
                    yaw_acquired = True

            if guided_active and not takeoff_sent and time.time() - start_time > 3:
                send_takeoff_command(mav, TAKEOFF_ALT)
                print(f"✓ 離陸指令（{TAKEOFF_ALT:.1f}m）")
                recording = True
                takeoff_sent = True

            if mode != 4:
                guided_active = False
                takeoff_sent = False
                recording = False
                takeoff_reached = False
                initial_target_set = False

            if not new_armed and recording:
                print("\n✓ ディスアーム検出 → 記録停止")
                running = False
                recording = False

        pos = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if pos:
            lat = pos.lat / 1e7
            lon = pos.lon / 1e7
            alt = pos.relative_alt / 1000

            if origin is None:
                origin = pos
                print(f"✓ 原点設定 lat={lat}, lon={lon}")

            # 簡易的なローカル座標変換
            x = (lon - origin.lon / 1e7) * 111319.5 * math.cos(math.radians(lat))
            y = (lat - origin.lat / 1e7) * 111319.5
            z = alt - (origin.relative_alt / 1000)

            with io_lock:
                gps_now['x'] = x
                gps_now['y'] = y
                gps_now['z'] = z

            # 離陸高度到達チェックと初期目標高さ設定
            if takeoff_sent and not takeoff_reached and z >= TAKEOFF_ALT * 0.9:
                takeoff_reached = True
                with io_lock:
                    target['z'] = TAKEOFF_ALT
                    initial_target_set = True
                print(f"✓ 離陸高度到達: {target['z']:.2f}m")

        time.sleep(1 / SEND_HZ)

# ───── 操作スレッド ─────
def control_loop(mav):
    global running, target, yaw_t_deg, initial_target_set

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    print("\n" + "="*60)
    print("キーボード制御モード（離陸高度自動設定→普通操作）")
    print("[u]南(-Y) [m]北(+Y) [h]東(+X) [l]西(-X) 10cm")
    print("[w]上昇(+Z) 10cm [z]下降(-Z) 10cm  [a/d]Yaw±5°")
    print("[t]基準点移動(70cm上空)  [q]終了")
    print("初期目標高度: 離陸高度自動設定後、上下移動")
    print("="*60)

    try:
        while running:
            key = get_key()
            moved = False
            echo = None

            if key == 'q':
                running = False
                break
            elif key == 'u':
                target['y'] -= STEP
                moved = True
                echo = "u South(-Y)"
            elif key == 'm':
                target['y'] += STEP
                moved = True
                echo = "m North(+Y)"
            elif key == 'h':
                target['x'] += STEP
                moved = True
                echo = "h East(+X)"
            elif key == 'l':
                target['x'] -= STEP
                moved = True
                echo = "l West(-X)"
            elif key == 'w':
                if initial_target_set:
                    target['z'] += STEP
                    moved = True
                    echo = "w Up(+10cm)"
                else:
                    echo = "w 待機中（離陸完了後に操作可能）"
            elif key == 'z':
                if initial_target_set and target['z'] - STEP >= 0.05:
                    target['z'] -= STEP
                    moved = True
                    echo = "z Down(-10cm)"
                elif not initial_target_set:
                    echo = "z 待機中（離陸完了後に操作可能）"
                else:
                    echo = "z 最低高度制限"
            elif key == 'a':
                yaw_t_deg = (yaw_t_deg - 5) % 360
                moved = True
                echo = "a Yaw-5"
            elif key == 'd':
                yaw_t_deg = (yaw_t_deg + 5) % 360
                moved = True
                echo = "d Yaw+5"
            elif key == 't':  # 基準点上空70cm移動
                if initial_target_set and origin:
                    ref_x, ref_y, ref_z = gps_to_local_xyz(36.0757693, 136.2132945, REF_ALT)
                    target['x'] = ref_x
                    target['y'] = ref_y
                    target['z'] = TARGET_HEIGHT_ABOVE_TAKEOFF  # 離陸高度を0基準として0.7m
                    moved = True
                    echo = f"t 基準点移動 X={target['x']:.2f} Y={target['y']:.2f} Z={target['z']:.2f}m"
                elif not origin:
                    echo = "t 原点未設定（GPS待機中）"
                else:
                    echo = "t 待機中（離陸完了後に操作可能）"

            if echo:
                sys.stdout.write(f"\x1b[2K\rKEY: {echo}\n")
                sys.stdout.flush()

            if moved:
                lat, lon, alt = local_xyz_to_gps(target['x'], target['y'], target['z'])
                send_setpoint(mav, int(lat * 1e7), int(lon * 1e7), alt, yaw_t_deg)
                print(f"  目標更新: X={target['x']:.2f} Y={target['y']:.2f} Z={target['z']:.2f}")

            time.sleep(0.05)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

# ───── データ記録スレッド ─────
def record_data():
    while running:
        if recording and origin:
            gps_x, gps_y, gps_z = gps_now['x'], gps_now['y'], gps_now['z']
            target_x, target_y, target_z = target['x'], target['y'], target['z']
            data_records.append([
                time.time(),
                gps_x, gps_y, gps_z,
                target_x, target_y, target_z
            ])
        time.sleep(1 / SEND_HZ)

# ───── CSV保存 ─────
def save_csv():
    if not data_records:
        print("⚠ 記録なし")
        return
    now = datetime.datetime.now(pytz.timezone("Asia/Tokyo")).strftime("%Y%m%d_%H%M%S")
    path = CSV_DIR / f"{now}_1.csv"
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Time', 'GPS_X', 'GPS_Y', 'GPS_Z', 'Target_X', 'Target_Y', 'Target_Z'])
        writer.writerows(data_records)
    print(f"\n✓ CSV保存完了: {path}")
    print(f"  記録行数: {len(data_records)}")

# ───── メイン関数 ─────
def main():
    global running
    signal.signal(signal.SIGINT, lambda sig, frame: setattr(sys.modules[__name__], "running", False))
    print("="*50)
    print("ArduPilot 精密制御 - 離陸高度自動設定版")
    print("初期目標高度: TAKEOFF_ALT 自動設定")
    print("その後: w/zキーで上下移動可能")
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
