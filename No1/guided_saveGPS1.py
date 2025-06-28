#!/usr/bin/env python3
"""
ArduPilot Guided制御 + GPS記録プログラム
- GPS目標位置を10Hzで送信
- GPS実位置を10Hzで記録
- プログラム終了時に日本時間_1.csvで保存
"""

import time, threading, csv, datetime, pytz, signal, sys
from pathlib import Path
from pymavlink import mavutil

# ===== 設定 =====
SEND_RATE = 10               # ArduPilotへの送信頻度 (Hz)
GPS_RATE = 10                # GPS記録頻度 (Hz)

CSV_DIR = Path.home() / "LOGS"
CSV_DIR.mkdir(exist_ok=True)

# ===== グローバル変数 =====
running = True
data_records = []
data_lock = threading.Lock()

# 現在状態
current_gps = {'lat': 0, 'lon': 0, 'alt': 0, 'time': 0}
current_target = {'lat': 0, 'lon': 0, 'alt': 0, 'time': 0}

def signal_handler(sig, frame):
    """Ctrl+Cでの終了処理"""
    global running
    print("\n終了処理中...")
    running = False

def connect_mavlink():
    """MAVLink接続"""
    m = mavutil.mavlink_connection('/dev/ttyAMA0', baud=1000000, rtscts=True)
    m.wait_heartbeat()
    print('✓ MAVLink接続完了')
    return m

def wait_guided_and_armed(m):
    """Guidedモード＆アーム待機"""
    print('Guidedモード待機中...')
    while running and m.recv_match(type='HEARTBEAT', blocking=True).custom_mode != 4:
        pass
    print('✓ Guidedモード')
    
    print('アーム待機中...')
    while running and not (m.recv_match(type='HEARTBEAT', blocking=True).base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
        pass
    print('✓ アーム完了')

def get_initial_position(m):
    """初期GPS位置取得"""
    print('GPS Fix待機中...')
    while running:
        gps = m.recv_match(type='GPS_RAW_INT', blocking=True)
        if gps and gps.fix_type >= 3:
            pos = m.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
            lat, lon, alt = pos.lat/1e7, pos.lon/1e7, pos.relative_alt/1000
            print(f"✓ 初期位置: {lat:.7f}, {lon:.7f}")
            return lat, lon, alt

def send_target_position(m, lat, lon, alt):
    """目標位置送信"""
    lat_int = int(lat * 1e7)
    lon_int = int(lon * 1e7)
    
    m.mav.set_position_target_global_int_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0x0FF8,  # 位置のみ制御
        lat_int, lon_int, alt,
        0, 0, 0, 0, 0, 0, 0, 0)

def gps_monitor_thread(m):
    """GPS監視スレッド（10Hz）"""
    global current_gps
    
    while running:
        pos = m.recv_match(type='GLOBAL_POSITION_INT', blocking=False, timeout=0.1)
        if pos:
            with data_lock:
                current_gps = {
                    'lat': pos.lat/1e7,
                    'lon': pos.lon/1e7,
                    'alt': pos.relative_alt/1000,
                    'time': time.time()
                }
        time.sleep(1/GPS_RATE)

def target_sender_thread(m):
    """目標位置送信スレッド（10Hz）"""
    while running:
        with data_lock:
            if current_target['lat'] != 0:
                send_target_position(m, current_target['lat'], 
                                   current_target['lon'], current_target['alt'])
        time.sleep(1/SEND_RATE)

def data_recorder_thread():
    """データ記録スレッド（10Hz）"""
    while running:
        with data_lock:
            if current_gps['time'] > 0 and current_target['time'] > 0:
                data_records.append([
                    time.time(),
                    current_gps['lat'], current_gps['lon'], current_gps['alt'],
                    current_target['lat'], current_target['lon'], current_target['alt']
                ])
        time.sleep(1/GPS_RATE)

def save_csv():
    """CSV保存"""
    if not data_records:
        print("記録データがありません")
        return
    
    # 日本時間でファイル名生成
    jst = pytz.timezone("Asia/Tokyo")
    timestamp = datetime.datetime.now(jst).strftime("%Y%m%d_%H%M%S")
    csv_path = CSV_DIR / f"{timestamp}_1.csv"
    
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['Time', 'GPS_Lat', 'GPS_Lon', 'GPS_Alt', 
                        'Target_Lat', 'Target_Lon', 'Target_Alt'])
        writer.writerows(data_records)
    
    print(f"✓ データ保存完了: {csv_path}")
    print(f"  記録行数: {len(data_records)}")

def main():
    global running
    
    # シグナルハンドラ設定
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        # 初期化
        m = connect_mavlink()
        wait_guided_and_armed(m)
        lat, lon, alt = get_initial_position(m)
        
        # 初期位置を目標位置として設定
        with data_lock:
            current_target['lat'] = lat
            current_target['lon'] = lon
            current_target['alt'] = alt
            current_target['time'] = time.time()
        
        print("制御開始...")
        
        # スレッド開始
        threads = [
            threading.Thread(target=gps_monitor_thread, args=(m,), daemon=True),
            threading.Thread(target=target_sender_thread, args=(m,), daemon=True),
            threading.Thread(target=data_recorder_thread, daemon=True)
        ]
        
        for t in threads:
            t.start()
        
        # メインループ（監視のみ）
        while running:
            time.sleep(1)
            print(f"記録数: {len(data_records)}", end='\r')
    
    except Exception as e:
        print(f"エラー: {e}")
    finally:
        running = False
        print("\nデータ保存中...")
        save_csv()
        print("プログラム終了")

if __name__ == "__main__":
    main()
