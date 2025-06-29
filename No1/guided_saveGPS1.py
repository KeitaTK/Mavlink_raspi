#!/usr/bin/env python3
"""
ArduPilot Guided制御 + GPS記録プログラム（修正版）
Guidedモードを待機し続ける版
"""

import time, threading, csv, datetime, pytz, signal, sys
from pathlib import Path
from pymavlink import mavutil

# ===== 設定 =====
SEND_RATE = 10               
GPS_RATE = 10                
TAKEOFF_ALT = 0.10           

CSV_DIR = Path.home() / "LOGS"
CSV_DIR.mkdir(exist_ok=True)

# ===== グローバル変数 =====
running = True
recording = False            
guided_active = False        
takeoff_sent = False         

data_records = []
data_lock = threading.Lock()

current_gps = {'lat': 0, 'lon': 0, 'alt': 0, 'time': 0}
current_target = {'lat': 0, 'lon': 0, 'alt': 0, 'time': 0}
current_mode = 0
armed = False

def signal_handler(sig, frame):
    """Ctrl+Cでの終了処理"""
    global running
    print("\n手動終了処理中...")
    running = False

def connect_mavlink():
    """MAVLink接続"""
    m = mavutil.mavlink_connection('/dev/ttyAMA0', baud=1000000, rtscts=True)
    m.wait_heartbeat()
    print('✓ MAVLink接続完了')
    return m

def set_msg_rate(m):
    """メッセージレート設定"""
    for mid, us in [(24,200000),(33,200000),(30,200000)]:
        m.mav.command_long_send(m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0, mid, us, 0,0,0,0,0)

def wait_for_arming(m):
    """アーム待機（ノンブロッキング）"""
    print('アーム待機中...')
    print('プロポでアームしてください')
    
    while running:
        heartbeat = m.recv_match(type='HEARTBEAT', blocking=False, timeout=0.1)
        if heartbeat:
            if heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                print('✓ アーム完了')
                return True
        time.sleep(0.1)
    return False

def wait_for_gps_fix(m):
    """GPS Fix待機（ノンブロッキング）"""
    print('GPS Fix待機中...')
    
    while running:
        gps_msg = m.recv_match(type='GPS_RAW_INT', blocking=False, timeout=0.1)
        if gps_msg and gps_msg.fix_type >= 3:
            pos = m.recv_match(type='GLOBAL_POSITION_INT', blocking=False, timeout=1.0)
            if pos:
                print(f"✓ GPS OK {pos.lat/1e7:.7f},{pos.lon/1e7:.7f}")
                return pos.lat/1e7, pos.lon/1e7, pos.relative_alt/1000
        time.sleep(0.5)
    return None, None, None

def send_takeoff_command(m, alt):
    """離陸コマンド送信"""
    print(f'離陸コマンド送信: {alt:.2f}m')
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, alt)

def send_target_position(m, lat, lon, alt):
    """目標位置送信"""
    lat_int = int(lat * 1e7)
    lon_int = int(lon * 1e7)
    
    m.mav.set_position_target_global_int_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0x0FF8,
        lat_int, lon_int, alt,
        0, 0, 0, 0, 0, 0, 0, 0)

def monitor_vehicle_state(m):
    """機体状態監視スレッド"""
    global current_mode, armed, guided_active, recording, takeoff_sent, current_gps
    
    while running:
        heartbeat = m.recv_match(type='HEARTBEAT', blocking=False, timeout=0.05)
        if heartbeat:
            current_mode = heartbeat.custom_mode
            armed = bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            
            # Guidedモード検出（モード4）
            if current_mode == 4 and armed and not guided_active:
                print('✓ Guidedモード検出 - 記録開始')
                guided_active = True
                recording = True
                
                if not takeoff_sent:
                    send_takeoff_command(m, TAKEOFF_ALT)
                    takeoff_sent = True
            
            # Guidedモードから外れた場合の処理
            if current_mode != 4 and guided_active:
                print('✓ Guidedモード終了')
                guided_active = False
                recording = False
            
            # ディスアーム検出（記録停止のみ、プログラム継続）
            if not armed and recording:
                print('✓ ディスアーム検出 - 記録停止')
                recording = False
                guided_active = False
                takeoff_sent = False  # リセット
        
        # GPS位置監視
        pos = m.recv_match(type='GLOBAL_POSITION_INT', blocking=False, timeout=0.05)
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
    """目標位置送信スレッド"""
    while running:
        if recording:  # 記録中のみ送信
            with data_lock:
                if current_target['lat'] != 0:
                    send_target_position(m, current_target['lat'],
                                       current_target['lon'], current_target['alt'])
        time.sleep(1/SEND_RATE)

def data_recorder_thread():
    """データ記録スレッド"""
    while running:
        if recording:
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
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        print("=" * 50)
        print("ArduPilot Guided制御 + GPS記録")
        print("=" * 50)
        
        m = connect_mavlink()
        set_msg_rate(m)
        time.sleep(2)
        
        if not wait_for_arming(m):
            print("アーム失敗")
            return
        
        lat, lon, alt = wait_for_gps_fix(m)
        if lat is None:
            print("GPS Fix失敗")
            return
        
        with data_lock:
            current_target['lat'] = lat
            current_target['lon'] = lon
            current_target['alt'] = TAKEOFF_ALT
            current_target['time'] = time.time()
        
        print("\n手順:")
        print("1. ✓ アーム完了")
        print("2. ✓ プログラム実行中")
        print("3. プロポのスイッチでGuidedモードにしてください")
        print("4. 自動で記録開始・離陸します")
        print("5. Ctrl+Cで終了（自動でCSV保存）")
        print("-" * 50)
        
        # スレッド開始
        threads = [
            threading.Thread(target=monitor_vehicle_state, args=(m,), daemon=True),
            threading.Thread(target=target_sender_thread, args=(m,), daemon=True),
            threading.Thread(target=data_recorder_thread, daemon=True)
        ]
        
        for t in threads:
            t.start()
        
        # メインループ（修正版：Ctrl+Cまで継続）
        while running:
            mode_name = {0:'Manual', 1:'Circle', 2:'Stabilize', 3:'Training', 
                        4:'Guided', 5:'LoiterUnlimited', 6:'RTL', 7:'Land', 
                        8:'Unknown', 9:'Drift', 10:'Sport'}.get(current_mode, f'Mode{current_mode}')
            
            status = "記録中" if recording else "待機中"
            record_count = len(data_records)
            
            print(f"\rモード: {mode_name}, アーム: {'Yes' if armed else 'No'}, "
                  f"状態: {status}, 記録数: {record_count}", end='')
            
            time.sleep(1)
    
    except Exception as e:
        print(f"\nエラー: {e}")
    finally:
        running = False
        print(f"\n最終記録数: {len(data_records)}")
        if data_records:
            print("データ保存中...")
            save_csv()
        print("プログラム終了")

if __name__ == "__main__":
    main()
