#!/usr/bin/env python3
import sys, select, time, math, threading, termios, tty, csv, datetime, signal
from pathlib import Path
from pymavlink import mavutil
from pyproj import Transformer
import pytz
import pyned2lla

# ───── ユーザー設定 ─────
STEP = 0.10          # 10cm移動
TAKEOFF_ALT = 0.50   # 初期離陸高度（m）
SEND_HZ = 10
CSV_DIR = Path.home() / "LOGS_Pixhawk6c"
CSV_DIR.mkdir(exist_ok=True)

# ───── 状態変数 ─────
running = True
recording = False
target = {'n':0.0, 'e':0.0, 'alt':TAKEOFF_ALT, 'yaw':180.0}
gps_now = {'lat':0, 'lon':0, 'alt':0}
data_records = []
origin = None
transformer = None
io_lock = threading.Lock()

# ───── ヨー角基準値関連 ─────
initial_yaw = None      # 離陸時の基準ヨー角
yaw_offset = 0.0        # 基準値からの相対角度
yaw_acquired = False    # ヨー角取得完了フラグ

# ───── 非ブロッキングキー入力 ─────
def get_key():
    if select.select([sys.stdin], [], [], 0)[0]:
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
        m.mav.command_long_send(m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0, mid, us, 0,0,0,0,0)

def send_takeoff_command(mav, alt):
    mav.mav.command_long_send(mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0,0,0,0,0,0,0, alt)

def send_target_position(mav, lat, lon, alt, yaw):
    mav.mav.set_position_target_global_int_send(
        0, mav.target_system, mav.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0x0FF8,  # Yaw有効
        int(lat * 1e7), int(lon * 1e7), alt,
        0,0,0, 0,0,0,
        math.radians(yaw), 0)

# ───── 離陸時ヨー角取得 ─────
def get_initial_yaw(mav):
    """離陸時の現在ヨー角を取得"""
    print("離陸時ヨー角取得中...")
    while True:
        att = mav.recv_match(type='ATTITUDE', blocking=True, timeout=1)
        if att:
            yaw = (math.degrees(att.yaw) + 360) % 360
            print(f"✓ 基準ヨー角設定: {yaw:.1f}°")
            return yaw

# ───── GPS更新 & ディスアーム監視 ─────
def monitor_vehicle(mav):
    global running, recording, gps_now, origin, transformer
    global initial_yaw, yaw_acquired
    guided_active = False
    armed = False
    takeoff_sent = False
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
                
                # 離陸前に基準ヨー角を取得
                if not yaw_acquired:
                    initial_yaw = get_initial_yaw(mav)
                    target['yaw'] = initial_yaw  # 基準値として設定
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

            if not new_armed and recording:
                print("\n✓ ディスアーム検出 → 記録停止")
                running = False
                recording = False

        pos = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if pos:
            gps_now = {
                'lat': pos.lat / 1e7,
                'lon': pos.lon / 1e7,
                'alt': pos.relative_alt / 1000
            }
            if origin is None:
                origin = pos
                init_transformer(pos.lat / 1e7, pos.lon / 1e7)
                print(f"✓ 原点設定 lat={gps_now['lat']}, lon={gps_now['lon']}")

        time.sleep(1 / SEND_HZ)

# ───── ENU → 緯度経度変換 ─────
def init_transformer(lat, lon):
    global transformer
    utm_zone = int((lon + 180) / 6) + 1
    proj_str = f"+proj=utm +zone={utm_zone} +datum=WGS84 +units=m +no_defs"
    transformer = Transformer.from_crs("EPSG:4326", proj_str, always_xy=True)

def enu_to_gps(n, e, alt):
    if origin is None:
        return gps_now['lat'], gps_now['lon'], alt
    wgs = pyned2lla.wgs84()
    lat0 = origin.lat / 1e7
    lon0 = origin.lon / 1e7
    alt0 = origin.alt / 1000
    latr, lonr, _ = pyned2lla.ned2lla(math.radians(lat0), math.radians(lon0), alt0, n, e, 0, wgs)
    return math.degrees(latr), math.degrees(lonr), alt

# ───── 操作スレッド ─────
def control_loop(mav):
    global running, target, yaw_offset, initial_yaw

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    print("キー操作: u/m/h/l/w/z/a/d (qで終了)")
    print("a/d: 基準ヨー角からの相対回転（±5°）")

    try:
        while running:
            key = get_key()
            moved = False
            if key == 'q':
                running = False
                break
            elif key == 'u': target['n'] -= STEP; moved = True
            elif key == 'm': target['n'] += STEP; moved = True
            elif key == 'h': target['e'] += STEP; moved = True
            elif key == 'l': target['e'] -= STEP; moved = True
            elif key == 'w': target['alt'] += STEP; moved = True
            elif key == 'z' and target['alt'] - STEP >= 0.05:
                target['alt'] -= STEP; moved = True
            elif key == 'a':
                # 基準ヨー角から左に5度
                yaw_offset -= 5
                if initial_yaw is not None:
                    target['yaw'] = (initial_yaw + yaw_offset) % 360
                    moved = True
            elif key == 'd':
                # 基準ヨー角から右に5度
                yaw_offset += 5
                if initial_yaw is not None:
                    target['yaw'] = (initial_yaw + yaw_offset) % 360
                    moved = True

            if moved:
                lat, lon, alt = enu_to_gps(target['n'], target['e'], target['alt'])
                send_target_position(mav, lat, lon, alt, target['yaw'])
                
                # 表示情報にオフセット値も追加
                if initial_yaw is not None:
                    print(f"\r→ 目標更新: N={target['n']:.2f} E={target['e']:.2f} Alt={target['alt']:.2f} "
                          f"Yaw={target['yaw']:.1f}° (基準{initial_yaw:.1f}°+{yaw_offset:.0f}°)", end='')
                else:
                    print(f"\r→ 目標更新: N={target['n']:.2f} E={target['e']:.2f} Alt={target['alt']:.2f} Yaw={target['yaw']:.1f}°", end='')

            time.sleep(0.05)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

# ───── データ記録スレッド ─────
def record_data():
    while running:
        if recording and origin:
            lat, lon, alt = gps_now['lat'], gps_now['lon'], gps_now['alt']
            x, y = gps_to_local(lat, lon)
            data_records.append([
                datetime.datetime.now(pytz.timezone("Asia/Tokyo")).isoformat(),
                lat, lon, alt,
                target['n'], target['e'], target['alt'], target['yaw'],
                x, y, yaw_offset, initial_yaw  # ヨー角情報も記録
            ])
        time.sleep(1 / SEND_HZ)

def gps_to_local(lat, lon):
    if transformer is None:
        return 0, 0
    x0, y0 = transformer.transform(origin.lon / 1e7, origin.lat / 1e7)
    x, y = transformer.transform(lon, lat)
    return x - x0, y - y0

def save_csv():
    if not data_records:
        print("⚠ 記録なし")
        return
    now = datetime.datetime.now(pytz.timezone("Asia/Tokyo")).strftime("%Y%m%d_%H%M%S")
    path = CSV_DIR / f"{now}_log.csv"
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Time', 'GPS_Lat', 'GPS_Lon', 'GPS_Alt',
                         'N(m)', 'E(m)', 'Target_Alt', 'Yaw(deg)',
                         'Local_X(m)', 'Local_Y(m)', 'Yaw_Offset(deg)', 'Initial_Yaw(deg)'])
        writer.writerows(data_records)
    print(f"\n✓ CSV保存完了: {path}")

# ───── メイン関数 ─────
def main():
    global running
    signal.signal(signal.SIGINT, lambda sig, frame: setattr(sys.modules[__name__], "running", False))
    print("="*50)
    print("ArduPilot Guided + SSH制御 + ローカル座標記録")
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
