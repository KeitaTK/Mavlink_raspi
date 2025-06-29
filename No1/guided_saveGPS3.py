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
target = {'n':0.0, 'e':0.0, 'alt':TAKEOFF_ALT}
gps_now = {'lat':0, 'lon':0, 'alt':0}
data_records = []
origin = None
transformer = None
io_lock = threading.Lock()

# ───── ヨー角制御関連 ─────
initial_yaw = None      # 離陸時の基準ヨー角（固定）
yaw_t_deg = 180.0       # 現在の目標ヨー角
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

# ───── 修正されたコマンド送信関数 ─────
def send_setpoint(m, lat_i, lon_i, alt):
    """位置制御のみ（ヨー角は無視）"""
    with io_lock:
        target.update(lat=lat_i/1e7, lon=lon_i/1e7, alt=alt)
    m.mav.set_position_target_global_int_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,  # bit10=1 yaw ignored, bit11=1 yaw rate ignored
        lat_i, lon_i, alt,
        0,0,0, 0,0,0,
        0, 0)  # yaw and yaw_rate ignored

def send_yaw_command(m, yaw_deg):
    """ヨー角制御専用（CONDITION_YAWコマンド）"""
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0, yaw_deg, 20, 0, 0, 0, 0, 0  # 20度/秒で回転
    )

# ───── 離陸時ヨー角取得 ─────
def get_initial_yaw(mav):
    """離陸時の現在ヨー角を取得して固定基準とする"""
    print("現在のヨー角取得中...")
    for _ in range(10):
        att = mav.recv_match(type='ATTITUDE', blocking=True, timeout=1)
        if att:
            yaw = (math.degrees(att.yaw) + 360) % 360
            print(f"✓ 基準ヨー角設定: {yaw:.1f}° (この角度を基準として保持)")
            return yaw
        time.sleep(0.1)
    
    print("⚠ ヨー角取得失敗、デフォルト値180°を使用")
    return 180.0

# ───── GPS更新 & ディスアーム監視 ─────
def monitor_vehicle(mav):
    global running, recording, gps_now, origin, transformer
    global initial_yaw, yaw_t_deg, yaw_acquired
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
                
                # 離陸前に基準ヨー角を取得・固定
                if not yaw_acquired:
                    initial_yaw = get_initial_yaw(mav)
                    yaw_t_deg = initial_yaw  # 初期値として設定
                    yaw_acquired = True
                    # 基準ヨー角をセット
                    send_yaw_command(mav, yaw_t_deg)

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

# ───── 操作スレッド（修正版） ─────
def control_loop(mav):
    global running, target, yaw_t_deg, initial_yaw

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)
    print("\n" + "="*60)
    print("キーボード制御モード（ヨー角固定方式）")
    print("[u]南 [m]北 [h]東 [l]西 10cm  [w/z]±10cm  [a/d]Yaw±5°  [q]終了")
    print("位置移動時：ヨー角は固定保持")
    print("ヨー角変更：キーボード入力のみ")
    print("="*60)

    try:
        while running:
            key = get_key()
            position_moved = False
            yaw_moved = False
            echo = None
            
            if key == 'q':
                running = False
                break
            # 位置制御（ヨー角は固定のまま）
            elif key == 'u': 
                target['n'] -= STEP; position_moved = True; echo = "u South"
            elif key == 'm': 
                target['n'] += STEP; position_moved = True; echo = "m North"
            elif key == 'h': 
                target['e'] += STEP; position_moved = True; echo = "h East"
            elif key == 'l': 
                target['e'] -= STEP; position_moved = True; echo = "l West"
            elif key == 'w': 
                target['alt'] += STEP; position_moved = True; echo = "w Up"
            elif key == 'z' and target['alt'] - STEP >= 0.05:
                target['alt'] -= STEP; position_moved = True; echo = "z Down"
            
            # ヨー角制御のみ（位置は変更しない）
            elif key == 'a':
                yaw_t_deg = (yaw_t_deg - 5) % 360; yaw_moved = True; echo = "a Yaw-5"
            elif key == 'd':
                yaw_t_deg = (yaw_t_deg + 5) % 360; yaw_moved = True; echo = "d Yaw+5"

            if echo:
                sys.stdout.write(f"\x1b[2K\rKEY: {echo}\n")
                sys.stdout.flush()

            # 位置コマンド送信（ヨー角は無視される）
            if position_moved:
                lat, lon, alt = enu_to_gps(target['n'], target['e'], target['alt'])
                send_setpoint(mav, int(lat * 1e7), int(lon * 1e7), alt)
                print(f"  位置更新: N={target['n']:.2f} E={target['e']:.2f} Alt={target['alt']:.2f} (ヨー角固定: {yaw_t_deg:.1f}°)")

            # ヨー角コマンド送信（位置は変更されない）
            if yaw_moved:
                send_yaw_command(mav, yaw_t_deg)
                if initial_yaw:
                    offset = (yaw_t_deg - initial_yaw + 180) % 360 - 180
                    print(f"  ヨー角変更: {yaw_t_deg:.1f}° (基準{initial_yaw:.1f}°から{offset:+.0f}°)")

            time.sleep(0.05)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)

# ───── データ記録スレッド ─────
def record_data():
    while running:
        if recording and origin:
            lat, lon, alt = gps_now['lat'], gps_now['lon'], gps_now['alt']
            x, y = gps_to_local(lat, lon)
            yaw_offset = (yaw_t_deg - initial_yaw + 180) % 360 - 180 if initial_yaw else 0
            data_records.append([
                datetime.datetime.now(pytz.timezone("Asia/Tokyo")).isoformat(),
                lat, lon, alt,
                target['n'], target['e'], target['alt'], yaw_t_deg,
                x, y, yaw_offset, initial_yaw
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
                         'N(m)', 'E(m)', 'Target_Alt', 'Target_Yaw(deg)',
                         'Local_X(m)', 'Local_Y(m)', 'Yaw_Offset(deg)', 'Initial_Yaw(deg)'])
        writer.writerows(data_records)
    print(f"\n✓ CSV保存完了: {path}")

# ───── メイン関数 ─────
def main():
    global running
    signal.signal(signal.SIGINT, lambda sig, frame: setattr(sys.modules[__name__], "running", False))
    print("="*50)
    print("ArduPilot 精密制御 - ヨー角固定方式")
    print("位置制御とヨー角制御を完全分離")
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
