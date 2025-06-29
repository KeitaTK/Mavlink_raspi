#!/usr/bin/env python3
import time, threading, csv, datetime, pytz, signal, sys
from pathlib import Path
from pymavlink import mavutil
from pyproj import Transformer

SEND_RATE = 10      # Hz
TAKEOFF_ALT = 0.5   # m
GUIDED_DELAY = 3.0  # s

CSV_DIR = Path.home() / "LOGS_Pixhawk6c"
CSV_DIR.mkdir(exist_ok=True)

# フラグ類
running = True
recording = False
guided_active = False
takeoff_sent = False

# 状態共有
data_records = []
data_lock = threading.Lock()

current_gps = {'lat': 0, 'lon': 0, 'alt': 0, 'time': 0}
current_target = {'lat': 0, 'lon': 0, 'alt': TAKEOFF_ALT, 'time': 0}
current_mode = 0
armed = False

# 座標変換用
origin_lat = None
origin_lon = None
transformer = None

def signal_handler(sig, frame):
    global running
    print("\n手動終了処理中...")
    running = False

def connect_mavlink():
    m = mavutil.mavlink_connection('/dev/ttyAMA0', baud=1000000, rtscts=True)
    m.wait_heartbeat()
    print('✓ MAVLink接続完了')
    return m

def set_msg_rate(m):
    for mid, us in [(24, 200000), (33, 200000), (30, 200000)]:
        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0, mid, us, 0, 0, 0, 0, 0
        )

def send_takeoff_command(m, alt):
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, alt
    )

def send_target_position(m, lat, lon, alt):
    m.mav.set_position_target_global_int_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0x0FF8,
        int(lat * 1e7), int(lon * 1e7), alt,
        0, 0, 0, 0, 0, 0, 0, 0
    )

def init_transformer(lat, lon):
    global transformer
    utm_zone = int((lon + 180) / 6) + 1
    proj_str = f"+proj=utm +zone={utm_zone} +datum=WGS84 +units=m +no_defs"
    transformer = Transformer.from_crs("EPSG:4326", proj_str, always_xy=True)

def gps_to_local(lat, lon):
    if transformer is None:
        return 0.0, 0.0
    x0, y0 = transformer.transform(origin_lon, origin_lat)
    x, y = transformer.transform(lon, lat)
    return x - x0, y - y0

def monitor_vehicle_state(m):
    global running, current_mode, armed, guided_active, takeoff_sent, recording
    global current_gps, origin_lat, origin_lon
    guided_start_time = 0

    while running:
        hb = m.recv_match(type='HEARTBEAT', blocking=False, timeout=0.05)
        if hb:
            current_mode = hb.custom_mode
            armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

            if current_mode == 4 and armed and not guided_active:
                print("✓ Guidedモード検出 → 安定化中...")
                guided_active = True
                guided_start_time = time.time()

            if guided_active and not takeoff_sent and (time.time() - guided_start_time) > GUIDED_DELAY:
                print(f"✓ 離陸指令（{TAKEOFF_ALT:.1f}m）")
                send_takeoff_command(m, TAKEOFF_ALT)
                recording = True
                takeoff_sent = True

            if current_mode != 4 and guided_active:
                print("✓ Guided終了 → 停止")
                guided_active = False
                recording = False
                takeoff_sent = False

            if not armed and recording:
                print("✓ ディスアーム検出 → 停止 & 保存")
                running = False
                recording = False

        pos = m.recv_match(type='GLOBAL_POSITION_INT', blocking=False, timeout=0.05)
        if pos:
            with data_lock:
                current_gps = {
                    'lat': pos.lat / 1e7,
                    'lon': pos.lon / 1e7,
                    'alt': pos.relative_alt / 1000,
                    'time': time.time()
                }
                if origin_lat is None:
                    origin_lat = current_gps['lat']
                    origin_lon = current_gps['lon']
                    init_transformer(origin_lat, origin_lon)
                    print(f"✓ 原点設定: lat={origin_lat}, lon={origin_lon}")

        time.sleep(1 / SEND_RATE)

def keyboard_input_thread():
    global current_target, running
    print("位置入力形式: 緯度,経度（例: 35.000123,135.000456）")
    print("`exit`で終了可能\n")

    while running:
        try:
            cmd = input("> ")
            if cmd.lower() == "exit":
                running = False
                break
            parts = cmd.strip().split(",")
            if len(parts) == 2:
                lat = float(parts[0])
                lon = float(parts[1])
                with data_lock:
                    current_target['lat'] = lat
                    current_target['lon'] = lon
                    current_target['time'] = time.time()
                print(f"✓ 目標更新: lat={lat}, lon={lon}")
            else:
                print("⚠ 入力形式エラー")
        except Exception as e:
            print(f"⚠ 入力エラー: {e}")

def target_sender():
    global mav
    while running:
        if recording:
            with data_lock:
                if current_target['time'] > 0:
                    send_target_position(mav, current_target['lat'],
                                         current_target['lon'],
                                         current_target['alt'])
        time.sleep(1 / SEND_RATE)

def data_recorder():
    while running:
        if recording:
            with data_lock:
                if current_gps['time'] > 0 and current_target['time'] > 0:
                    data_records.append([
                        datetime.datetime.now(pytz.timezone("Asia/Tokyo")).isoformat(),
                        current_gps['lat'], current_gps['lon'], current_gps['alt'],
                        current_target['lat'], current_target['lon'], current_target['alt']
                    ])
        time.sleep(1 / SEND_RATE)

def save_csv():
    if not data_records:
        print("記録データがありません")
        return
    jst = pytz.timezone("Asia/Tokyo")
    timestamp = datetime.datetime.now(jst).strftime("%Y%m%d_%H%M%S")
    csv_path = CSV_DIR / f"{timestamp}_1.csv"

    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['JST_Time',
                         'GPS_Lat', 'GPS_Lon', 'GPS_Alt',
                         'Target_Lat', 'Target_Lon', 'Target_Alt',
                         'Local_X', 'Local_Y', 'Target_X', 'Target_Y'])
        for row in data_records:
            _, glat, glon, galt, tlat, tlon, talt = row
            gx, gy = gps_to_local(glat, glon)
            tx, ty = gps_to_local(tlat, tlon)
            writer.writerow(row + [gx, gy, tx, ty])
    print(f"✓ CSV保存完了: {csv_path}, 記録数: {len(data_records)}")

def main():
    global mav
    signal.signal(signal.SIGINT, signal_handler)
    print("=" * 50)
    print("ArduPilot Guided + SSH制御 + ローカル座標記録")
    print("=" * 50)

    mav = connect_mavlink()
    set_msg_rate(mav)

    print("プロポでアーム後、Guidedにしてください")

    threads = [
        threading.Thread(target=monitor_vehicle_state, args=(mav,), daemon=True),
        threading.Thread(target=target_sender, daemon=True),
        threading.Thread(target=data_recorder, daemon=True),
        threading.Thread(target=keyboard_input_thread, daemon=True)
    ]

    for t in threads:
        t.start()

    while running:
        mode_name = {0: 'Manual', 1: 'Circle', 2: 'Stabilize', 3: 'Training',
                     4: 'Guided', 5: 'LoiterUnlimited', 6: 'RTL', 7: 'Land',
                     9: 'Drift', 10: 'Sport'}.get(current_mode, f'Mode{current_mode}')
        print(f"\rモード: {mode_name}, アーム: {'Yes' if armed else 'No'}, "
              f"記録: {'中' if recording else '待機'}, データ数: {len(data_records)}", end='')
        time.sleep(1)

    print("\n記録終了、保存処理中...")
    save_csv()
    print("プログラム終了")

if __name__ == "__main__":
    main()
