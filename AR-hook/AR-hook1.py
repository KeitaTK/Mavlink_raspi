#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os,sys,select,time,math,threading,termios,tty,csv,datetime,signal
from pathlib import Path
import numpy as np
import cv2
from cv2 import aruco
# pyrefly: ignore [missing-import]
from pymavlink import mavutil
import pytz
# Raspberry Pi 5 カメラサポート
try:
    # pyrefly: ignore [missing-import]
    from picamera2 import Picamera2
    HAS_PICAMERA2 = True
except ImportError:
    HAS_PICAMERA2 = False
# ───── ユーザー設定 ─────
STEP = 0.10
TAKEOFF_ALT = 0.50
SEND_HZ = 10
MASK = 0x09F8
# ───── ArUco・カメラ追跡設定 ─────
MARKER_SIZE = 0.04  # マーカーの一辺の長さ（メートル、4cm）
ID_CENTER_MARKER = 1  # 正方形の中心に配置されるマーカーID（ID 1）
SQUARE_MARKER_IDS = [2, 3, 4, 5]  # 正方形4頂点のマーカーID（ID2〜ID5）
SQUARE_SIDE = 0.15  # 正方形の一辺長（メートル）
HALF_SIDE = SQUARE_SIDE / 2
MARKER_CENTER_OFFSETS = {
    2: np.array([ +HALF_SIDE, +HALF_SIDE, 0.0], dtype=np.float32),  # 右上
    3: np.array([ -HALF_SIDE, +HALF_SIDE, 0.0], dtype=np.float32),  # 左上
    4: np.array([ -HALF_SIDE, -HALF_SIDE, 0.0], dtype=np.float32),  # 左下
    5: np.array([ +HALF_SIDE, -HALF_SIDE, 0.0], dtype=np.float32),  # 右下
}
# ───── 基準点設定 ─────
REF_LAT, REF_LON, REF_ALT = 36.0757800, 136.2132900, 0.0
TARGET_HEIGHT_ABOVE_TAKEOFF = 1.10
CSV_DIR = Path.home() / "LOGS_Pixhawk6c"
CSV_DIR.mkdir(exist_ok=True)
# ───── 状態変数 ─────
running = True
recording = True  # ← 常に記録
target = {'x':0.0, 'y':0.0, 'z':0.0}
gps_now = {'x':0.0, 'y':0.0, 'z':0.0}
current_yaw_deg = 180.0  # MAVLinkから取得する実時間ヨー角（度）
data_records = []
origin = None
io_lock = threading.Lock()
initial_target_set = False
initial_yaw, yaw_t_deg, yaw_acquired = None, 180.0, False
guided_mode_active = False  # Guidedモード判定用
# 最後に確定した荷物中心（カメラ座標系）を保持
last_center_cam = None
# ───── キー入力処理 ─────
def get_key():
    if select.select([sys.stdin], [], [], 0):
        return sys.stdin.read(1)
    return None
# ───── MAVLink接続と設定 ─────
def input_with_timeout(prompt, timeout=5, default='1'):
    print(prompt, end='', flush=True)
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        try:
            return sys.stdin.readline().strip()
        except Exception:
            return default
    else:
        print(f"\n[タイムアウト] {timeout}秒間入力がなかったため、デフォルト値 '{default}' を使用します。")
        return default

def connect_mavlink():
    print("\n" + "="*50)
    print(" 接続方法を選択してください")
    print(" 1: Serial接続 (/dev/ttyAMA0, 1000000 baud, RTS/CTS有効) ※デフォルト")
    print(" 2: USB接続 (/dev/ttyACM0, 115200 baud)")
    print("="*50)
    
    choice = input_with_timeout("選択 (1 または 2): ", timeout=5, default='1')
    
    if choice == '2':
        device = '/dev/ttyACM0'
        baud = 115200
        rtscts = False
    else:
        device = '/dev/ttyAMA0'
        baud = 1000000
        rtscts = True

    print(f"\n✓ MAVLink接続を開始します: {device} (BaudRate: {baud}, RTS/CTS: {rtscts})")
    m = mavutil.mavlink_connection(device, baud=baud, rtscts=rtscts)
    
    print("ハートビート信号を待機中 (タイムアウト: 10秒)...")
    hb = m.wait_heartbeat(timeout=10)
    if hb is None:
        print("❌ ハートビートの受信タイムアウト: Pixhawkからの応答がありません。")
        print("配線やPixhawkの電源、BaudRate設定を確認してください。")
        sys.exit(1)
        
    print(f"✓ MAVLink接続完了 (System ID: {m.target_system}, Component ID: {m.target_component})")
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
# ───── カメラ座標系から世界絶対座標系（東・北・上）への座標変換 ─────
def camera_to_world_xyz(tvec, yaw_deg, gps_drone):
    """
    下向きカメラ座標系の相対ベクトル [x_cam, y_cam, z_cam] を、
    機体ヨー角（yaw_deg）を用いて世界絶対座標系（East-North-Up）の目標値に変換します。
    """
    # 下向きカメラ座標系の機体ボディ座標系へのマッピング：
    # 画像の右方向（Camera +X） -> 機体の右方向（Body +Y）
    # 画像の下方向（Camera +Y） -> 機体の後方向（Body -X） => 前方向（Body +X） = -Camera Y
    x_body = -tvec[1]
    y_body = tvec[0]
    
    # ヨー角（ラジアン）
    yaw_rad = math.radians(yaw_deg)
    
    # ボディ座標系（前・右）から世界絶対座標系（東・北）への回転変換
    # 東（x） = gps_now['x'] + x_body * sin(yaw) + y_body * cos(yaw)
    # 北（y） = gps_now['y'] + x_body * cos(yaw) - y_body * sin(yaw)
    dx = x_body * math.sin(yaw_rad) + y_body * math.cos(yaw_rad)
    dy = x_body * math.cos(yaw_rad) - y_body * math.sin(yaw_rad)
    
    target_x = gps_drone['x'] + dx
    target_y = gps_drone['y'] + dy
    target_z = gps_drone['z'] - tvec[2]  # カメラZ軸は下向きなので、z方向のオフセットを計算
    
    return target_x, target_y, target_z
# ───── ポーズ推定関数 ─────
def my_estimatePoseSingleMarkers(corners, marker_size, camera_matrix, distortion_coeff):
    marker_points = np.array([
        [-marker_size / 2,  marker_size / 2, 0],
        [ marker_size / 2,  marker_size / 2, 0],
        [ marker_size / 2, -marker_size / 2, 0],
        [-marker_size / 2, -marker_size / 2, 0]
    ], dtype=np.float32)
    rvecs = []
    tvecs = []
    for c in corners:
        c = np.array(c, dtype=np.float32).reshape(4, 2)
        success, rvec, tvec = cv2.solvePnP(
            marker_points,
            c,
            camera_matrix,
            distortion_coeff,
            flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        rvecs.append(rvec)
        tvecs.append(tvec)
    return rvecs, tvecs

def estimate_square_center_from_marker(corner, marker_id, marker_size, camera_matrix, distortion_coeff):
    rvecs, tvecs = my_estimatePoseSingleMarkers([corner], marker_size, camera_matrix, distortion_coeff)
    rvec = rvecs[0].reshape(3)
    tvec = tvecs[0].reshape(3)
    # ID1 (ドローン側) はここで特別扱いしない。
    # ここでは常にマーカー位置（または頂点オフセット適用後の中心推定）を返す。
    offset = MARKER_CENTER_OFFSETS.get(marker_id)
    if offset is None:
        # マーカーが正方形の頂点リストにない場合は、そのまま tvec を返す（例: ID1など）
        return tvec
    R, _ = cv2.Rodrigues(rvec)
    return tvec + R.dot(offset)

# ───── 状態監視スレッド ─────
def monitor_vehicle(m):
    global running, gps_now, origin, initial_yaw, yaw_t_deg, yaw_acquired, initial_target_set, guided_mode_active, current_yaw_deg
    guided, armed, takeoff_sent, takeoff_reached = False, False, False, False
    start_time = 0
    while running:
        hb = m.recv_match(type='HEARTBEAT', blocking=False, timeout=0.05)
        if hb:
            is_guided = hb.custom_mode == 4
            new_armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            guided_mode_active = is_guided  # ← 状態を記録用に更新
            if is_guided and new_armed and not guided:
                print("✓ Guidedモード検出 → 離陸準備")
                start_time, guided = time.time(), True
                if not yaw_acquired:
                    initial_yaw = get_initial_yaw(m)
                    yaw_t_deg, yaw_acquired = initial_yaw, True
            if guided and not takeoff_sent and time.time() - start_time > 3:
                send_takeoff_command(m, TAKEOFF_ALT)
                print(f"✓ 離陸指令（{TAKEOFF_ALT:.1f}m）")
                takeoff_sent = True
            if not is_guided:
                guided, takeoff_sent, takeoff_reached, initial_target_set = False, False, False, False
            if armed and not new_armed:  # アームされていた状態からディスアームされた場合のみ終了
                print("\n✓ ディスアーム検出")
                running = False  # プログラム停止（記録は最後に保存）
            armed = new_armed  # アーム状態を更新
        # 実時間の機体姿勢(ヨー角)情報の取得
        att = m.recv_match(type='ATTITUDE', blocking=False)
        if att:
            with io_lock:
                current_yaw_deg = (math.degrees(att.yaw) + 360) % 360
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
# ───── バックグラウンド・カメラ追跡スレッド ─────
def camera_tracker_loop(m, show_window=False):
    global running, target, gps_now, current_yaw_deg, initial_target_set, guided_mode_active, last_center_cam
    
    print("カメラ初期化中（解像度: 720p, 画面表示: なし）...")
    picam2 = None
    cap = None
    if HAS_PICAMERA2:
        print("Raspberry Pi Camera (Picamera2) を使用します。")
        try:
            picam2 = Picamera2()
            config = picam2.create_preview_configuration(main={"format": 'BGR888', "size": (640, 480)})
            picam2.configure(config)
            picam2.start()
            print("✓ Picamera2 起動完了 (720p)")
        except Exception as e:
            print(f"⚠ Picamera2 の起動に失敗しました: {e}。USBカメラへの切り替えを試みます。")
            picam2 = None
    if picam2 is None:
        print("USBカメラ (VideoCapture) を起動します。")
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("❌ エラー: カメラを開けませんでした。追跡スレッドを停止します。")
            return
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        print("✓ USBカメラ 起動完了 (640x480)")
    # カメラキャリブレーションパラメータの読み込み
    params_file = "camera_params.npz"
    camera_matrix = np.array([
        [786.38858756,   0.,         351.02240753],
        [0.,            788.85699087,260.48178893],
        [0.,              0.,          1.]
    ])
    distortion_coeff = np.array([-0.03169828, -0.16365523, 0.0051104, -0.00278013, 0.50978427])
    if os.path.exists(params_file):
        try:
            with np.load(params_file) as data:
                camera_matrix = data["mtx"]
                distortion_coeff = data["dist"]
            print("✓ カメラパラメータのロードに成功しました。")
        except Exception as e:
            print(f"⚠ カメラパラメータのロードに失敗しました ({e})。デフォルト値を使用します。")
    else:
        print("⚠ camera_params.npz が見つかりません。デフォルトのキャリブレーション値を使用します。")
    # ArUcoディテクタの設定
    dictionary = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    detector = aruco.ArucoDetector(dictionary)
    print("✓ マーカー追跡ループを開始します（中心ID: {}, 正方形頂点ID: {}）...".format(ID_CENTER_MARKER, SQUARE_MARKER_IDS))
    last_print_time = 0
    try:
        while running:
            # フレーム取得
            if picam2:
                img = picam2.capture_array()
                ret = True
            else:
                ret, img = cap.read()
            if not ret or img is None:
                time.sleep(0.01)
                continue
            # マーカー検出
            corners, ids, rejected = detector.detectMarkers(img)
            if ids is not None:
                center_estimates = []
                detected_ids = []
                id1_cam = None
                for i, corner in enumerate(corners):
                    marker_id = int(ids[i][0])
                    # ID2-5: 荷物の頂点として中心を推定
                    if marker_id in MARKER_CENTER_OFFSETS:
                        center_cam = estimate_square_center_from_marker(
                            corner, marker_id, MARKER_SIZE, camera_matrix, distortion_coeff
                        )
                        if center_cam is not None:
                            center_estimates.append(center_cam)
                            detected_ids.append(marker_id)
                    # ID1: ドローン側マーカー（補正用）
                    elif marker_id == ID_CENTER_MARKER:
                        # ID1のカメラ座標系位置を取得しておく
                        id1_cam = estimate_square_center_from_marker(corner, marker_id, MARKER_SIZE, camera_matrix, distortion_coeff)

                used_last_center = False
                if center_estimates:
                    # 荷物中心のカメラ座標系推定（複数検出なら平均化）
                    center_cam = np.mean(center_estimates, axis=0)
                    # 最終中心を更新
                    last_center_cam = center_cam
                else:
                    # 頂点マーカーが見えないがID1のみ見えている場合は最後に記録した中心を利用
                    if id1_cam is not None and last_center_cam is not None:
                        center_cam = last_center_cam
                        used_last_center = True
                    else:
                        # 中心もID1もない場合は更新しない
                        continue

                # テレメトリおよび姿勢（Yaw角）データの取得
                with io_lock:
                    drone_gps = gps_now.copy()
                    drone_yaw = current_yaw_deg

                # ID1が見えていれば、中心との差分を使ってドローン目標を算出
                if id1_cam is not None:
                    tvec_error = center_cam - id1_cam
                else:
                    tvec_error = center_cam

                # カメラ座標から世界絶対座標系（東・北）に変換して目標位置を算出
                target_x, target_y, target_z = camera_to_world_xyz(tvec_error, drone_yaw, drone_gps)

                # グローバル目標値の更新（高さは固定）
                with io_lock:
                    target['x'] = target_x
                    target['y'] = target_y
                    target['z'] = TARGET_HEIGHT_ABOVE_TAKEOFF

                # 自動で誘導目標コマンドを送信
                if guided_mode_active and initial_target_set:
                    lat, lon, alt = local_xyz_to_gps(target_x, target_y, TARGET_HEIGHT_ABOVE_TAKEOFF)
                    send_setpoint(m, int(lat*1e7), int(lon*1e7), alt, yaw_t_deg)

                # 1秒間隔でコンソールに進捗を表示
                if time.time() - last_print_time > 1.0:
                    ids_text = ",".join(str(mid) for mid in detected_ids)
                    id1_text = "(ID1 vis)" if id1_cam is not None else ""
                    last_text = "(using last center)" if used_last_center else ""
                    print(f"[Tracker] 検出IDs: {ids_text} {id1_text} {last_text} | 推定中心 [X:{center_cam[0]:.2f}, Y:{center_cam[1]:.2f}, Z:{center_cam[2]:.2f}] | 目標座標 [X:{target_x:.2f}, Y:{target_y:.2f}]")
                    last_print_time = time.time()
            # 表示が許可されていればウィンドウ表示
            if show_window:
                display = img.copy()
                if ids is not None:
                    display = aruco.drawDetectedMarkers(display, corners, ids)
                cv2.imshow("AR Camera", display)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    running = False
                    break

            # 処理負荷抑制のための微小なスリープ
            time.sleep(0.01)
    finally:
        print("カメラ追跡スレッドを終了し、カメラを解放します...")
        if cap:
            cap.release()
        if picam2:
            try:
                picam2.stop()
            except:
                pass
        if show_window:
            cv2.destroyAllWindows()
# ───── キーボード操作 ─────
def control_loop(m):
    global running, target, yaw_t_deg, initial_target_set
    fd, old = sys.stdin.fileno(), termios.tcgetattr(sys.stdin.fileno())
    tty.setcbreak(fd)
    try:
        print("\n" + "="*60)
        print("キーボード制御モード（u/m/h/l:XY, w/z:Z, a/d:Yaw, t:基準点）")
        print("※ ArUco追跡動作中は、マーカー検出により目標が自動更新されます")
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
            time.sleep(0.05)  # 100% CPU使用率を防ぐためにスリープを追加
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
# ───── データ記録 ─────
def record_data():
    while running:
        with io_lock:
            gps = gps_now.copy()
            tgt = target.copy()  # 機上テスト等でも追跡値を確認できるよう、Guidedモード成否に関わらず常に目標値を記録します
        data_records.append([time.time(), gps['x'], gps['y'], gps['z'], tgt['x'], tgt['y'], tgt['z']])
        time.sleep(1 / SEND_HZ)
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
    print(f"\n✓ CSV保存完了: {path} ({len(data_records)} 行)")
# ───── メイン関数 ─────
def main():
    global running
    print("="*50)
    print("ArduPilot 精密制御 - バックグラウンド追跡常時記録モード")
    print("="*50)
    
    try:
        mav = connect_mavlink()
        set_msg_rate(mav)
        
        # スレッド起動
        threading.Thread(target=monitor_vehicle, args=(mav,), daemon=True).start()
        threading.Thread(target=record_data, daemon=True).start()
        # 起動時にカメラ映像表示の有無を選択
        choice = input_with_timeout("カメラ映像を表示しますか？ 1:表示 2:非表示 (デフォルト2): ", timeout=5, default='2')
        show_camera_window = (choice.strip() == '1')
        threading.Thread(target=camera_tracker_loop, args=(mav, show_camera_window), daemon=True).start()
        
        # メインコントロール（キーボード制御）
        control_loop(mav)
        
    except KeyboardInterrupt:
        print("\n⚠ ユーザーによる強制終了 (Ctrl+C) を検出しました。")
    finally:
        running = False
        print("\n記録終了、CSV保存中...")
        save_csv()
        print("✓ プログラム終了")
if __name__ == "__main__":
    main()

