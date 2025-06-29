#!/usr/bin/env python3
"""
ArduPilot Guided制御 + GPS記録 + キーボード操作プログラム
  • u/m/h/l : 10 cm  南 / 北 / 東 / 西
  • w / z   : 10 cm  上昇 / 下降
  • a / d   : Yaw ±5°
  • q : 終了
  • 離陸時のヨー角は離陸前のものを保持
"""

import sys, select, time, math, threading, termios, tty, csv, datetime, pytz, signal
from pathlib import Path
import pyned2lla
from pymavlink import mavutil

# ===== 設定 =====
STEP = 0.10                  # 10cm移動
TAKEOFF_ALT = 0.5            # 離陸高度 1m
GPS_HZ = 5                   # 表示更新頻度
SEND_RATE = 10               # 目標位置送信頻度
GUIDED_DELAY = 3.0           # Guidedモード安定化時間
MASK = 0x09F8                # ヨー角有効、ヨーレート無視

CSV_DIR = Path.home() / "LOGS"
CSV_DIR.mkdir(exist_ok=True)

# ===== グローバル変数 =====
running = True
recording = False
guided_active = False
takeoff_sent = False
keyboard_active = False

# 位置・姿勢状態
gps_now = {'lat': 0, 'lon': 0, 'alt': 0}
target = {'lat': 0, 'lon': 0, 'alt': 0}
yaw_now = 0.0
yaw_t_deg = 0.0              # 離陸前ヨー角で初期化
initial_yaw = 0.0            # 離陸前のヨー角を保存

# 制御状態
current_mode = 0
armed = False
data_records = []

# スレッド制御
io_lock = threading.Lock()
data_lock = threading.Lock()

def signal_handler(sig, frame):
    """Ctrl+Cでの終了処理"""
    global running
    print("\n手動終了処理中...")
    running = False

# ───── 非ブロッキングキー入力 ─────
def get_key():
    """非ブロッキングキーボード入力"""
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None

# ───── MAVLink接続・設定 ─────
def connect_mavlink():
    """MAVLink接続"""
    m = mavutil.mavlink_connection('/dev/ttyAMA0', baud=1000000, rtscts=True)
    m.wait_heartbeat()
    print('✓ MAVLink接続完了')
    return m

def set_msg_rate(m):
    """メッセージレート設定"""
    for mid, us in [(24,200000),(33,200000),(30,200000)]:   # 5 Hz
        m.mav.command_long_send(m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0, mid, us, 0,0,0,0,0)

# ───── 初期化関数 ─────
def wait_for_arming(m):
    """アーム待機"""
    print('アーム待機中... プロポでアームしてください')
    while running:
        heartbeat = m.recv_match(type='HEARTBEAT', blocking=False, timeout=0.1)
        if heartbeat and (heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED):
            print('✓ アーム完了')
            return True
        time.sleep(0.1)
    return False

def wait_for_gps_fix(m):
    """GPS Fix待機"""
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

def get_current_yaw(m):
    """現在のヨー角取得（離陸前の基準ヨー角として使用）"""
    print('現在のヨー角取得中...')
    while running:
        att = m.recv_match(type='ATTITUDE', blocking=False, timeout=0.1)
        if att:
            current_yaw = (math.degrees(att.yaw) + 360) % 360
            print(f'✓ 現在のヨー角: {current_yaw:.1f}° （これを基準とします）')
            return current_yaw
        time.sleep(0.1)
    return 0.0

# ───── コマンド送信 ─────
def send_takeoff_command(m, alt):
    """離陸コマンド送信"""
    print(f'離陸コマンド送信: {alt:.2f}m')
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, alt)

def send_setpoint(m, lat_i, lon_i, alt, yaw_deg):
    """目標位置・姿勢送信"""
    with io_lock:
        target.update(lat=lat_i/1e7, lon=lon_i/1e7, alt=alt)
    
    m.mav.set_position_target_global_int_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        MASK,
        lat_i, lon_i, alt,
        0,0,0, 0,0,0,
        math.radians(yaw_deg), 0)

# ───── 監視・制御スレッド ─────
def vehicle_monitor_thread(m):
    """機体状態監視・表示スレッド"""
    global current_mode, armed, guided_active, recording, takeoff_sent, yaw_now
    global gps_now, keyboard_active
    guided_start_time = 0
    
    while running:
        # ハートビート監視
        heartbeat = m.recv_match(type='HEARTBEAT', blocking=False, timeout=0.05)
        if heartbeat:
            current_mode = heartbeat.custom_mode
            armed = bool(heartbeat.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            
            # Guidedモード検出
            if current_mode == 4 and armed and not guided_active:
                print('✓ Guidedモード検出 - 安定化待機中...')
                guided_active = True
                recording = True
                guided_start_time = time.time()
                
            # 離陸コマンド送信
            if (guided_active and not takeoff_sent and 
                time.time() - guided_start_time >= GUIDED_DELAY):
                print(f'✓ Guidedモード安定 - 離陸コマンド送信: {TAKEOFF_ALT:.1f}m')
                send_takeoff_command(m, TAKEOFF_ALT)
                takeoff_sent = True
                
                # 離陸後しばらくしてキーボード制御開始
                threading.Timer(5.0, lambda: setattr(sys.modules[__name__], 'keyboard_active', True)).start()
                print('5秒後にキーボード制御が有効になります')
            
            # モード終了処理
            if current_mode != 4 and guided_active:
                print('✓ Guidedモード終了')
                guided_active = False
                recording = False
                keyboard_active = False
                takeoff_sent = False
                guided_start_time = 0
        
        # GPS・姿勢監視
        pos = m.recv_match(type='GLOBAL_POSITION_INT', blocking=False, timeout=0.05)
        att = m.recv_match(type='ATTITUDE', blocking=False, timeout=0.05)
        
        if att:
            yaw_now = (math.degrees(att.yaw) + 360) % 360
            
        if pos:
            with io_lock:
                gps_now.update(lat=pos.lat/1e7, lon=pos.lon/1e7, alt=pos.relative_alt/1000)
                
                # 表示更新（キーボード制御中のみ）
                if keyboard_active and target['lat']:
                    l1 = f"GPS:    {gps_now['lat']:.7f}, {gps_now['lon']:.7f}, Alt: {gps_now['alt']:.3f} m, Yaw: {yaw_now:6.1f}°"
                    l2 = f"TARGET: {target['lat']:.7f}, {target['lon']:.7f}, Alt: {target['alt']:.3f} m, Yaw: {yaw_t_deg:6.1f}°"
                    sys.stdout.write("\x1b[2K\r"+l1+"\n")
                    sys.stdout.write("\x1b[2K"+l2+"\r")
                    sys.stdout.flush()
        
        time.sleep(1/GPS_HZ)

def data_recorder_thread():
    """データ記録スレッド"""
    while running:
        if recording:
            with data_lock:
                if gps_now['lat'] != 0 and target['lat'] != 0:
                    data_records.append([
                        time.time(),
                        gps_now['lat'], gps_now['lon'], gps_now['alt'],
                        target['lat'], target['lon'], target['alt']
                    ])
        time.sleep(1/SEND_RATE)

# ───── キーボード制御ループ ─────
def keyboard_control_loop(m, origin_lat, origin_lon, origin_alt):
    """キーボード制御メインループ"""
    global yaw_t_deg, keyboard_active
    
    # 座標変換準備
    wgs = pyned2lla.wgs84()
    lat0r, lon0r = math.radians(origin_lat), math.radians(origin_lon)
    n = e = 0.0
    alt = TAKEOFF_ALT
    
    # ターミナル設定
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setraw(fd)
    
    print("\n" + "="*60)
    print("キーボード制御開始")
    print("[u]南 [m]北 [h]東 [l]西 10cm  [w/z]±10cm  [a/d]Yaw±5°  [q]終了")
    print("="*60)
    
    # 初期位置設定
    send_setpoint(m, int(origin_lat*1e7), int(origin_lon*1e7), alt, yaw_t_deg)
    
    try:
        while running and keyboard_active:
            k = get_key()
            moved = False
            echo = None
            
            if k == 'q': 
                break
            elif k == 'u': 
                n -= STEP; moved = True; echo = "u South"
            elif k == 'm': 
                n += STEP; moved = True; echo = "m North"
            elif k == 'h': 
                e += STEP; moved = True; echo = "h East"
            elif k == 'l': 
                e -= STEP; moved = True; echo = "l West"
            elif k == 'w': 
                alt += STEP; moved = True; echo = "w Up"
            elif k == 'z':
                if alt - STEP >= 0.05: 
                    alt -= STEP; moved = True; echo = "z Down"
            elif k == 'a': 
                yaw_t_deg = (yaw_t_deg - 5) % 360; moved = True; echo = "a Yaw-5"
            elif k == 'd': 
                yaw_t_deg = (yaw_t_deg + 5) % 360; moved = True; echo = "d Yaw+5"
            
            if echo:
                sys.stdout.write("\x1b[2K\rKEY: " + echo + "\n")
                sys.stdout.flush()
            
            if moved:
                # NED → LLA変換
                latr, lonr, _ = pyned2lla.ned2lla(lat0r, lon0r, origin_alt, n, e, 0, wgs)
                send_setpoint(m,
                    int(math.degrees(latr) * 1e7),
                    int(math.degrees(lonr) * 1e7),
                    alt, yaw_t_deg)
            
            time.sleep(0.05)
            
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        print('\nキーボード制御終了')

# ───── CSV保存 ─────
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

# ───── メイン関数 ─────
def main():
    global running, yaw_t_deg, initial_yaw
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        print("=" * 50)
        print("ArduPilot Guided制御 + GPS記録 + キーボード操作")
        print("=" * 50)
        
        # 初期化
        m = connect_mavlink()
        set_msg_rate(m)
        time.sleep(2)
        
        # 手順実行
        if not wait_for_arming(m):
            return
        
        lat, lon, alt = wait_for_gps_fix(m)
        if lat is None:
            return
        
        # 離陸前ヨー角取得・保存
        initial_yaw = get_current_yaw(m)
        yaw_t_deg = initial_yaw  # 基準ヨー角として設定
        
        # 初期目標位置設定
        with data_lock:
            target.update(lat=lat, lon=lon, alt=TAKEOFF_ALT)
        
        print("\n手順:")
        print("1. ✓ アーム完了")
        print("2. ✓ GPS Fix完了")
        print(f"3. ✓ 基準ヨー角設定: {initial_yaw:.1f}°")
        print("4. プロポのスイッチでGuidedモードにしてください")
        print("5. 自動で記録開始・離陸・キーボード制御開始")
        print("-" * 50)
        
        # スレッド開始
        monitor_thread = threading.Thread(target=vehicle_monitor_thread, args=(m,), daemon=True)
        recorder_thread = threading.Thread(target=data_recorder_thread, daemon=True)
        
        monitor_thread.start()
        recorder_thread.start()
        
        # Guidedモード・キーボード制御待機
        while running:
            if keyboard_active:
                keyboard_control_loop(m, lat, lon, alt)
                break
            
            mode_name = {0:'Manual', 4:'Guided'}.get(current_mode, f'Mode{current_mode}')
            status = "記録中" if recording else "待機中"
            
            print(f"\rモード: {mode_name}, 状態: {status}, 記録数: {len(data_records)}", end='')
            time.sleep(1)
    
    except Exception as e:
        print(f"\nエラー: {e}")
    finally:
        running = False
        print(f"\n最終記録数: {len(data_records)}")
        if data_records:
            save_csv()
        print("プログラム終了")

if __name__ == "__main__":
    main()
