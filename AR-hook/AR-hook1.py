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
CAMERA_WIDTH = 1640
CAMERA_HEIGHT = 1232
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
current_roll_rad = 0.0   # MAVLinkから取得する実時間ロール角（ラジアン）
current_pitch_rad = 0.0  # MAVLinkから取得する実時間ピッチ角（ラジアン）
current_yaw_rad = 0.0    # MAVLinkから取得する実時間ヨー角（ラジアン）
current_yaw_deg = 180.0  # MAVLinkから取得する実時間ヨー角（度）
data_records = []
origin = None
io_lock = threading.Lock()
initial_target_set = False
initial_yaw, yaw_t_deg, yaw_acquired = None, 180.0, False
guided_mode_active = False  # Guidedモード判定用
# 最後に確定した荷物中心と回転ベクトル（カメラ座標系）を保持
last_center_cam = None
last_rvec_cargo = None
cargo_center_world = {'x':0.0, 'y':0.0, 'z':0.0}
cargo_detected = False
id1_detected = False
dist_id1_to_cargo_x = float('nan')
dist_id1_to_cargo_y = float('nan')
dist_id1_to_cargo_z = float('nan')
cargo_center_cam_x = float('nan')
cargo_center_cam_y = float('nan')

# ───── 新規追加: Motive UDP受信スレッド ─────
import socket, struct
MOTIVE_FORMAT = '<BiiiHffffd'  # 39バイト
motive_roll_rad = 0.0
motive_pitch_rad = 0.0
motive_yaw_rad = 0.0
motive_attitude_received = False

def quaternion_to_euler_ned(qx, qy, qz, qw):    # Motive(X=北,Z=東,Y=上左手系) -> NED右手系変換済みquat
    roll  = math.atan2(2*(qw*qx+qy*qz), 1-2*(qx**2+qy**2))
    pitch = math.asin(max(-1,min(1, 2*(qw*qy-qz*qx))))
    yaw   = math.atan2(2*(qw*qz+qx*qy), 1-2*(qy**2+qz**2))
    return roll, pitch, yaw

def motive_udp_listener():
    global motive_roll_rad, motive_pitch_rad, motive_yaw_rad, motive_attitude_received
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    except AttributeError:
        pass
    
    try:
        sock.bind(('0.0.0.0', 15769))
    except Exception as e:
        print(f"[Motive UDP] Bind failed: {e}")
        return

    sock.settimeout(0.5)
    print("[Motive UDP] Receiver thread started on port 15769")

    while running:
        try:
            data, addr = sock.recvfrom(1024)
            if len(data) >= 39:
                unpacked = struct.unpack(MOTIVE_FORMAT, data[:39])
                rigid_body_id = unpacked[0]
                if rigid_body_id == 1:
                    motive_qx = unpacked[5]
                    motive_qy = unpacked[6]
                    motive_qz = unpacked[7]
                    motive_qw = unpacked[8]
                    
                    # Convert Motive quat (X=North, Y=Up, Z=East, left-handed) to NED quat (X=North, Y=East, Z=Down, right-handed)
                    ned_qx = motive_qx
                    ned_qy = motive_qz
                    ned_qz = -motive_qy
                    ned_qw = motive_qw
                    
                    r, p, y = quaternion_to_euler_ned(ned_qx, ned_qy, ned_qz, ned_qw)
                    
                    with io_lock:
                        motive_roll_rad = r
                        motive_pitch_rad = p
                        motive_yaw_rad = y
                        motive_attitude_received = True
        except socket.timeout:
            continue
        except Exception as e:
            print(f"[Motive UDP] Error: {e}")
            time.sleep(0.1)
            
    sock.close()
    print("[Motive UDP] Receiver thread stopped")

# ───── キー入力処理 ─────
def get_key():
    if select.select([sys.stdin], [], [], 0):
        return sys.stdin.read(1)
    return None
# ───── MAVLink接続と設定 ─────
def input_with_timeout(prompt, timeout=5, default='1'):
    try:
        termios.tcflush(sys.stdin.fileno(), termios.TCIFLUSH)
    except Exception:
        pass
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
def camera_to_world_xyz(tvec, roll_rad, pitch_rad, yaw_rad, gps_drone):
    """
    下向きカメラ座標系の相対ベクトル [x_cam, y_cam, z_cam] を、
    機体姿勢（Roll, Pitch, Yaw）の3D回転行列を用いて世界絶対座標系（East-North-Up）の目標値に変換します。
    """
    # 1. カメラ座標系 -> 機体ボディ座標系（FRD: Front-Right-Down）
    # x_body = -y_cam, y_body = x_cam, z_body = z_cam
    p_body = np.array([-tvec[1], tvec[0], tvec[2]], dtype=np.float32)
    
    # 2. ロール、ピッチ、ヨーの回転行列を計算
    cr, sr = math.cos(roll_rad), math.sin(roll_rad)
    cp, sp = math.cos(pitch_rad), math.sin(pitch_rad)
    cy, sy = math.cos(yaw_rad), math.sin(yaw_rad)
    
    # R = Rz(yaw) * Ry(pitch) * Rx(roll)
    R_body_to_ned = np.array([
        [cp*cy,  sr*sp*cy - cr*sy,  cr*sp*cy + sr*sy],
        [cp*sy,  sr*sp*sy + cr*cy,  cr*sp*sy - sr*cy],
        [-sp,    sr*cp,            cr*cp]
    ], dtype=np.float32)
    
    # 機体ボディ座標から地球座標（NED）に変換
    p_ned = R_body_to_ned.dot(p_body)
    
    # 3. NED（北・東・下）から ENU（東・北・上）へ変換
    dx_enu = p_ned[1]  # East = Y_NED
    dy_enu = p_ned[0]  # North = X_NED
    dz_enu = -p_ned[2] # Up = -Z_NED
    
    target_x = gps_drone['x'] + dx_enu
    target_y = gps_drone['y'] + dy_enu
    target_z = gps_drone['z'] + dz_enu
    
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
        # 実時間の機体姿勢(ロール、ピッチ、ヨー角)情報の取得
        att = m.recv_match(type='ATTITUDE', blocking=False)
        if att:
            with io_lock:
                current_roll_rad = att.roll
                current_pitch_rad = att.pitch
                current_yaw_rad = att.yaw
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
    global running, target, gps_now, current_yaw_deg, initial_target_set, guided_mode_active, last_center_cam, last_rvec_cargo
    global cargo_center_world, cargo_detected, id1_detected, dist_id1_to_cargo_x, dist_id1_to_cargo_y, dist_id1_to_cargo_z
    global cargo_center_cam_x, cargo_center_cam_y
    
    print(f"カメラ初期化中（解像度: {CAMERA_WIDTH}x{CAMERA_HEIGHT}）...")
    picam2 = None
    cap = None
    video_writer = None
    if HAS_PICAMERA2:
        print("Raspberry Pi Camera (Picamera2) を使用します。")
        try:
            picam2 = Picamera2()
            config = picam2.create_preview_configuration(main={"format": 'BGR888', "size": (CAMERA_WIDTH, CAMERA_HEIGHT)})
            picam2.configure(config)
            picam2.start()
            print(f"✓ Picamera2 起動完了 ({CAMERA_WIDTH}x{CAMERA_HEIGHT})")
        except Exception as e:
            print(f"⚠ Picamera2 の起動に失敗しました: {e}。USBカメラへの切り替えを試みます。")
            picam2 = None
    if picam2 is None:
        print("USBカメラ (VideoCapture) を起動します。")
        cap = cv2.VideoCapture(0)
        if not cap.isOpened():
            print("❌ エラー: カメラを開けませんでした。追跡スレッドを停止します。")
            return
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        print(f"✓ USBカメラ 起動完了 ({CAMERA_WIDTH}x{CAMERA_HEIGHT})")
    # カメラキャリブレーションパラメータの読み込み
    params_file = "camera_params.npz"
    # デフォルト値（640x360用）を新解像度に合わせてスケール調整
    camera_matrix = np.array([
        [786.38858756 * (CAMERA_WIDTH / 640.0),   0.,                                     351.02240753 * (CAMERA_WIDTH / 640.0)],
        [0.,                                    788.85699087 * (CAMERA_HEIGHT / 360.0),  260.48178893 * (CAMERA_HEIGHT / 360.0)],
        [0.,                                    0.,                                     1.]
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
            has_cargo = False
            has_id1 = False
            dist_x, dist_y, dist_z = float('nan'), float('nan'), float('nan')
            
            if ids is not None:
                obj_points = []
                img_points = []
                detected_ids = []
                id1_cam = None
                for i, corner in enumerate(corners):
                    marker_id = int(ids[i][0])
                    # ID2-5: 荷物の頂点として中心を推定
                    if marker_id in MARKER_CENTER_OFFSETS:
                        O_id = MARKER_CENTER_OFFSETS[marker_id]
                        # 各マーカーの4頂点の3D座標（荷物座標系）
                        corners_3d = np.array([
                            [O_id[0] - MARKER_SIZE / 2, O_id[1] + MARKER_SIZE / 2, 0.0],
                            [O_id[0] + MARKER_SIZE / 2, O_id[1] + MARKER_SIZE / 2, 0.0],
                            [O_id[0] + MARKER_SIZE / 2, O_id[1] - MARKER_SIZE / 2, 0.0],
                            [O_id[0] - MARKER_SIZE / 2, O_id[1] - MARKER_SIZE / 2, 0.0],
                        ], dtype=np.float32)
                        obj_points.append(corners_3d)
                        img_points.append(corner.reshape(4, 2))
                        detected_ids.append(marker_id)
                    # ID1: ドローン側マーカー（補正用）
                    elif marker_id == ID_CENTER_MARKER:
                        # ID1のカメラ座標系位置を取得しておく
                        id1_cam = estimate_square_center_from_marker(corner, marker_id, MARKER_SIZE, camera_matrix, distortion_coeff)

                used_last_center = False
                center_cam = None
                rvec_cargo = None
                
                if len(obj_points) > 0:
                    obj_points = np.vstack(obj_points)
                    img_points = np.vstack(img_points)
                    # 全検出マーカーの頂点から荷物中心位置・姿勢をPnP推定
                    success, rvec_sol, tvec_sol = cv2.solvePnP(
                        obj_points,
                        img_points,
                        camera_matrix,
                        distortion_coeff,
                        flags=cv2.SOLVEPNP_ITERATIVE
                    )
                    if success:
                        center_cam = tvec_sol.reshape(3)
                        rvec_cargo = rvec_sol.reshape(3)
                        last_center_cam = center_cam
                        last_rvec_cargo = rvec_cargo
                        has_cargo = True
                else:
                    # 頂点マーカーが見えないがID1のみ見えている場合は最後に記録した中心情報を利用
                    if id1_cam is not None and last_center_cam is not None and last_rvec_cargo is not None:
                        center_cam = last_center_cam
                        rvec_cargo = last_rvec_cargo
                        used_last_center = True
                        has_cargo = True

                if id1_cam is not None:
                    has_id1 = True

                if has_cargo:
                    # テレメトリおよび姿勢（Roll, Pitch, Yaw）データの取得
                    with io_lock:
                        drone_gps = gps_now.copy()
                        if motive_attitude_received:
                            drone_roll = motive_roll_rad
                            drone_pitch = motive_pitch_rad
                            drone_yaw = motive_yaw_rad
                        else:
                            drone_roll = current_roll_rad
                            drone_pitch = current_pitch_rad
                            drone_yaw = current_yaw_rad

                    # 荷物中心の世界座標を算出
                    cargo_x, cargo_y, cargo_z = camera_to_world_xyz(center_cam, drone_roll, drone_pitch, drone_yaw, drone_gps)

                    # ID1が見えていれば、中心との差分を使ってドローン目標を算出、および相対xyz座標を計算
                    if has_id1:
                        tvec_error = center_cam - id1_cam
                        # 中心位置を原点とする座標系から見たID1までのxyz距離
                        v_cam = id1_cam - center_cam
                        R_cargo, _ = cv2.Rodrigues(rvec_cargo)
                        v_cargo = R_cargo.T.dot(v_cam)
                        dist_x = float(v_cargo[0])
                        dist_y = float(v_cargo[1])
                        dist_z = float(v_cargo[2])
                    else:
                        # ID1が見えない場合は荷物中心そのものを目標にする（ただしID1目標距離は算出しない）
                        tvec_error = center_cam

                    # カメラ座標から世界絶対座標系（東・北）に変換して目標位置を算出
                    target_x, target_y, target_z = camera_to_world_xyz(tvec_error, drone_roll, drone_pitch, drone_yaw, drone_gps)

                    # グローバル目標値等の更新
                    with io_lock:
                        target['x'] = target_x
                        target['y'] = target_y
                        target['z'] = TARGET_HEIGHT_ABOVE_TAKEOFF
                        
                        cargo_center_world['x'] = cargo_x
                        cargo_center_world['y'] = cargo_y
                        cargo_center_world['z'] = cargo_z
                        cargo_detected = True
                        id1_detected = has_id1
                        dist_id1_to_cargo_x = dist_x
                        dist_id1_to_cargo_y = dist_y
                        dist_id1_to_cargo_z = dist_z
                        # 荷物中心(center_cam)の画像上のピクセル座標(u, v)を計算し、画面中心からのずれ(px)を求める
                        proj_pts, _ = cv2.projectPoints(
                            np.array([[0.0, 0.0, 0.0]], dtype=np.float32), 
                            np.zeros(3, dtype=np.float32), 
                            center_cam.reshape(3), 
                            camera_matrix, 
                            distortion_coeff
                        )
                        u, v = proj_pts[0][0]
                        H, W = img.shape[:2]
                        cargo_center_cam_x = u - W / 2.0
                        cargo_center_cam_y = v - H / 2.0

                    # 自動で誘導目標コマンドを送信
                    if guided_mode_active and initial_target_set:
                        lat, lon, alt = local_xyz_to_gps(target_x, target_y, TARGET_HEIGHT_ABOVE_TAKEOFF)
                        send_setpoint(m, int(lat*1e7), int(lon*1e7), alt, yaw_t_deg)

                    # 1秒間隔でコンソールに進捗を表示
                    if time.time() - last_print_time > 1.0:
                        ids_text = ",".join(str(mid) for mid in detected_ids)
                        id1_text = "(ID1 vis)" if has_id1 else "(ID1 NOT vis)"
                        last_text = "(using last center)" if used_last_center else ""
                        
                        dist_id1_text = f"X:{dist_x:.3f}, Y:{dist_y:.3f}, Z:{dist_z:.3f}" if has_id1 else "N/A"
                        print(f"[Tracker] 検出IDs: {ids_text} {id1_text} {last_text} | "
                               f"推定中心(px): [X:{cargo_center_cam_x:.1f}, Y:{cargo_center_cam_y:.1f}] | "
                               f"ID1-中心目標距離(xyz): [{dist_id1_text}] | "
                               f"目標座標 [X:{target_x:.2f}, Y:{target_y:.2f}]")
                        last_print_time = time.time()
                else:
                    # 荷物が検出されていない場合
                    with io_lock:
                        cargo_detected = False
                        id1_detected = has_id1
                        dist_id1_to_cargo_x = float('nan')
                        dist_id1_to_cargo_y = float('nan')
                        dist_id1_to_cargo_z = float('nan')
                        cargo_center_cam_x = float('nan')
                        cargo_center_cam_y = float('nan')
            else:
                # ids is None
                with io_lock:
                    cargo_detected = False
                    id1_detected = False
                    dist_id1_to_cargo_x = float('nan')
                    dist_id1_to_cargo_y = float('nan')
                    dist_id1_to_cargo_z = float('nan')
                    cargo_center_cam_x = float('nan')
                    cargo_center_cam_y = float('nan')

            # 表示が許可されていればウィンドウ表示
            if show_window:
                display = img.copy()
                if ids is not None:
                    display = aruco.drawDetectedMarkers(display, corners, ids)
                    
                    if has_cargo and center_cam is not None and rvec_cargo is not None:
                        # 1. 中心位置の投影と描画 (緑の丸)
                        center_pts_3d = np.array([[0.0, 0.0, 0.0]], dtype=np.float32)
                        center_img_pts, _ = cv2.projectPoints(
                            center_pts_3d, rvec_cargo, center_cam, camera_matrix, distortion_coeff
                        )
                        center_pixel = tuple(center_img_pts[0][0].astype(int))
                        cv2.circle(display, center_pixel, 8, (0, 255, 0), 2, lineType=cv2.LINE_AA)
                        cv2.circle(display, center_pixel, 2, (0, 255, 0), -1, lineType=cv2.LINE_AA)
                        cv2.putText(display, f"Cargo Center (px): ({cargo_center_cam_x:.1f}, {cargo_center_cam_y:.1f})", 
                                    (center_pixel[0] + 10, center_pixel[1] - 10),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

                        # 2. 座標系の描画 (X:赤, Y:緑, Z:青)
                        try:
                            cv2.drawFrameAxes(display, camera_matrix, distortion_coeff, rvec_cargo, center_cam, SQUARE_SIDE)
                        except AttributeError:
                            try:
                                cv2.aruco.drawAxis(display, camera_matrix, distortion_coeff, rvec_cargo, center_cam, SQUARE_SIDE)
                            except AttributeError:
                                pass

                        # 3. ID2-5がすべて検出されている場合、15cm四方の枠を描画
                        if len([mid for mid in [2, 3, 4, 5] if mid in detected_ids]) == 4:
                            square_pts_3d = np.array([
                                [-HALF_SIDE,  HALF_SIDE, 0.0],
                                [ HALF_SIDE,  HALF_SIDE, 0.0],
                                [ HALF_SIDE, -HALF_SIDE, 0.0],
                                [-HALF_SIDE, -HALF_SIDE, 0.0]
                            ], dtype=np.float32)
                            square_img_pts, _ = cv2.projectPoints(
                                square_pts_3d, rvec_cargo, center_cam, camera_matrix, distortion_coeff
                            )
                            pts_2d = square_img_pts.reshape(-1, 2).astype(np.int32)
                            cv2.polylines(display, [pts_2d], isClosed=True, color=(0, 165, 255), thickness=2, lineType=cv2.LINE_AA)
                            cv2.putText(display, "15cm Cargo Frame", (pts_2d[0][0], pts_2d[0][1] - 10),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 1, cv2.LINE_AA)

                        # 4. 画面上にID1の相対位置を表示
                        if has_id1:
                            text_str = f"ID1 Rel to Center: X:{dist_x:.3f} Y:{dist_y:.3f} Z:{dist_z:.3f}"
                            cv2.putText(display, text_str, (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2, cv2.LINE_AA)
                        else:
                            cv2.putText(display, "ID1 Rel to Center: N/A (ID1 invisible)", (15, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)

                if video_writer is None:
                    # ディスプレイ用にウィンドウの設定を行う (1640x1232は大きいため縮小表示)
                    cv2.namedWindow("AR Camera", cv2.WINDOW_NORMAL)
                    cv2.resizeWindow("AR Camera", 820, 616)
                    
                    height, width = display.shape[:2]
                    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                    now_str = datetime.datetime.now(pytz.timezone("Asia/Tokyo")).strftime("%Y%m%d_%H%M%S")
                    video_path = CSV_DIR / f"{now_str}_video.mp4"
                    video_writer = cv2.VideoWriter(str(video_path), fourcc, 20.0, (width, height))
                    print(f"✓ ビデオ録画を開始しました: {video_path}")
                video_writer.write(display)

                cv2.imshow("AR Camera", display)
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    running = False
                    break

            # 処理負荷抑制のための微小なスリープ
            time.sleep(0.01)
    finally:
        print("カメラ追跡スレッドを終了し、カメラを解放します...")
        if video_writer is not None:
            video_writer.release()
            print("✓ ビデオ録画を終了しました。")
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
            cargo_w = cargo_center_world.copy()
            c_det = cargo_detected
            i1_det = id1_detected
            d_i1_c_x = dist_id1_to_cargo_x
            d_i1_c_y = dist_id1_to_cargo_y
            d_i1_c_z = dist_id1_to_cargo_z
            c_cam_x = cargo_center_cam_x
            c_cam_y = cargo_center_cam_y
        data_records.append([
            time.time(), 
            gps['x'], gps['y'], gps['z'], 
            tgt['x'], tgt['y'], tgt['z'],
            cargo_w['x'], cargo_w['y'], cargo_w['z'],
            1 if c_det else 0,
            1 if i1_det else 0,
            d_i1_c_x,
            d_i1_c_y,
            d_i1_c_z,
            c_cam_x,
            c_cam_y
        ])
        time.sleep(1 / SEND_HZ)
def save_csv():
    if not data_records:
        print("⚠ 記録なし")
        return
    now = datetime.datetime.now(pytz.timezone("Asia/Tokyo")).strftime("%Y%m%d_%H%M%S")
    path = CSV_DIR / f"{now}_1.csv"
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'Time', 'GPS_X', 'GPS_Y', 'GPS_Z', 'Target_X', 'Target_Y', 'Target_Z',
            'Cargo_X', 'Cargo_Y', 'Cargo_Z', 'Cargo_Detected', 'ID1_Detected',
            'ID1_to_Cargo_DX', 'ID1_to_Cargo_DY', 'ID1_to_Cargo_DZ',
            'Est_Center_Cam_X', 'Est_Center_Cam_Y'
        ])
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
        threading.Thread(target=motive_udp_listener, daemon=True).start()
        # 起動時にカメラ映像表示の有無を選択
        choice = input_with_timeout("カメラ映像を表示しますか？ 1:表示 2:非表示 (デフォルト2): ", timeout=15, default='2')
        show_camera_window = (choice.strip() == '1')
        
        # コントロールループをサブスレッドで起動 (キー入力を別スレッドに逃がす)
        control_thread = threading.Thread(target=control_loop, args=(mav,), daemon=False)
        control_thread.start()
        
        # カメラ追跡ループをメインスレッドで実行 (OpenCV GUIのメインスレッド制限対策)
        camera_tracker_loop(mav, show_camera_window)
        
    except KeyboardInterrupt:
        print("\n⚠ ユーザーによる強制終了 (Ctrl+C) を検出しました。")
    finally:
        running = False
        # コントロールスレッドの終了待機
        if 'control_thread' in locals() and control_thread.is_alive():
            control_thread.join()
        print("\n記録終了、CSV保存中...")
        save_csv()
        print("✓ プログラム終了")
if __name__ == "__main__":
    main()

