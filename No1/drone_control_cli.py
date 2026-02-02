#!/usr/bin/env python3
"""
ドローン制御CLIアプリケーション（NED座標系）
- 離陸地点自動記録・離陸
- wasd平行移動（w=南、s=北、a=東、d=西）
- 上下矢印で上下移動（↑=上、↓=下）
- 5cm移動（通常）/ 20cm移動（Shift）
- RTH（離陸地点直上に移動して着陸）
- ヨー軸常に南向き（180度）
- 座標系：NED（North-East-Down）
"""
import sys, select, time, math, threading, termios, tty, signal
from pathlib import Path
from pymavlink import mavutil
import pytz

# ───── 設定 ─────
TAKEOFF_ALT = 0.50       # 離陸高度（m）
SEND_HZ = 10             # 更新周期（Hz）
MASK = 0x09F8            # bit10=0(Yaw有効) bit11=1(YawRate無視)
YAW_SOUTH = 180.0        # ヨー角固定（南向き）

# ───── 速度制御設定 ─────
STEP_VELOCITY = 0.8      # [m/s] ステップ入力の大きさ（40cm/s）
MASK_VELOCITY = 0b0000110111000111  # velocityのみ有効 (pos無視, accel無視, yaw無視)


# ───── 状態変数 ─────
running = True
target = {'x': 0.0, 'y': 0.0, 'z': 0.0}       # ローカル座標目標（NED系：X=北、Y=東、Z=下）
gps_now = {'x': 0.0, 'y': 0.0, 'z': 0.0}      # 現在位置（NED系）
takeoff_point = None                          # 離陸地点（GPS）
rth_to_land = False                           # RTH着陸フラグ
saved_position = None                         # 記録位置（NED系）
origin = None
io_lock = threading.Lock()
initial_target_set = False
guided_mode_active = False
armed = False
mav = None
takeoff_sent = False
takeoff_reached = False


# ───── MAVLink接続と設定 ─────
def connect_mavlink():
    """MAVLink接続"""
    m = mavutil.mavlink_connection('/dev/ttyAMA0', baud=1000000, rtscts=True)
    m.wait_heartbeat()
    print("✓ MAVLink接続完了")
    return m


def set_msg_rate(m):
    """メッセージレート設定"""
    for mid, us in [(24, 200000), (33, 200000), (30, 200000)]:
        m.mav.command_long_send(
            m.target_system, m.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0, mid, us, 0, 0, 0, 0, 0
        )


def send_takeoff_command(m, alt):
    """離陸コマンド送信"""
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0, 0, 0, 0, 0, 0, 0, alt
    )


def send_setpoint(m, lat_i, lon_i, alt, yaw_deg):
    """目標位置送信（Yaw制御込み）"""
    m.mav.set_position_target_global_int_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        MASK,
        lat_i, lon_i, alt,
        0, 0, 0, 0, 0, 0,
        math.radians(yaw_deg), 0
    )
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_CONDITION_YAW,
        0, yaw_deg, 20, 0, 0, 0, 0, 0
    )


def send_land_command(m):
    """着陸コマンド送信"""
    m.mav.command_long_send(
        m.target_system, m.target_component,
        mavutil.mavlink.MAV_CMD_NAV_LAND,
        0, 0, 0, 0, 0, 0, 0, 0
    )


def send_velocity_step(m, vx, vy, vz):
    """速度指令を送信する関数 (NED座標系)
    vx: North [m/s]
    vy: East  [m/s]
    vz: Down  [m/s]
    """
    m.mav.set_position_target_local_ned_send(
        0,                          # time_boot_ms
        m.target_system,            # target_system
        m.target_component,         # target_component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        MASK_VELOCITY,              # type_mask (速度のみ有効)
        0, 0, 0,                    # x, y, z (位置：無視)
        vx, vy, vz,                 # vx, vy, vz (速度：ここが重要)
        0, 0, 0,                    # afx, afy, afz (加速度：無視)
        0, 0                        # yaw, yaw_rate (無視)
    )


# ───── 非ブロッキングキー入力 ─────
def get_key():
    """非ブロッキングでキー入力を取得（矢印キー対応）"""
    if select.select([sys.stdin], [], [], 0):
        ch = sys.stdin.read(1)
        # エスケープシーケンス検出（矢印キー）
        if ch == '\x1b':
            if select.select([sys.stdin], [], [], 0.001):
                ch2 = sys.stdin.read(1)
                if ch2 == '[':
                    if select.select([sys.stdin], [], [], 0.001):
                        ch3 = sys.stdin.read(1)
                        if ch3 == 'A': return 'UP'
                        elif ch3 == 'B': return 'DOWN'
        return ch
    return None


# ───── 座標変換 ─────
def local_xyz_to_gps(x, y, z):
    """ローカル座標（NED系） → GPS座標
    NED系: X=北、Y=東、Z=下（負が上）
    """
    if origin is None:
        return 0.0, 0.0, 0.0
    lat0 = origin.lat / 1e7
    lon0 = origin.lon / 1e7
    alt0 = origin.relative_alt / 1000
    # NED系からGPS座標への変換
    lat = lat0 + x / 111319.5  # X=北方向
    lon = lon0 + y / (111319.5 * math.cos(math.radians(lat0)))  # Y=東方向
    alt = alt0 - z  # Z=下（負が上なので符号反転）
    return lat, lon, alt


# ───── 車両状態監視スレッド ─────
def monitor_vehicle():
    """車両状態監視（GPS、アーム状態、Guidedモード検出）"""
    global running, gps_now, origin, initial_target_set, guided_mode_active, armed
    global takeoff_sent, takeoff_reached, rth_to_land

    while running:
        # ハートビート監視
        hb = mav.recv_match(type='HEARTBEAT', blocking=False, timeout=0.05)
        if hb:
            mode = hb.custom_mode
            new_armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            guided_mode_active = (mode == 4)
            armed = new_armed



            # Guidedモード解除で初期化
            if mode != 4:
                takeoff_sent = False
                takeoff_reached = False
                initial_target_set = False

        # GPS位置更新
        pos = mav.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
        if pos:
            lat = pos.lat / 1e7
            lon = pos.lon / 1e7
            alt = pos.relative_alt / 1000

            if origin is None:
                origin = pos
                print(f"✓ 原点設定 lat={lat}, lon={lon}")

            # GPS座標 → NED系ローカル座標変換
            # X=北、Y=東、Z=下（負が上）
            lat0 = origin.lat / 1e7
            lon0 = origin.lon / 1e7
            alt0 = origin.relative_alt / 1000
            x = (lat - lat0) * 111319.5  # X=北方向
            y = (lon - lon0) * 111319.5 * math.cos(math.radians(lat0))  # Y=東方向
            z = -(alt - alt0)  # Z=下（高度が上がると負になる）

            with io_lock:
                gps_now['x'] = x
                gps_now['y'] = y
                gps_now['z'] = z

            # 離陸高度到達検出（Z<0で上昇）
            if takeoff_sent and not takeoff_reached and z <= -TAKEOFF_ALT * 0.9:
                takeoff_reached = True
                with io_lock:
                    target['z'] = -TAKEOFF_ALT  # NED系でZ負が上
                    initial_target_set = True
                print(f"✓ 離陸高度到達: {-target['z']:.2f}m（高度）")

            # RTH着陸判定（原点付近で低高度になったら着陸）
            if rth_to_land and math.sqrt(x**2 + y**2) < 0.5 and z > -TAKEOFF_ALT - 0.2:
                print("✓ RTH完了 → 着陸コマンド送信")
                send_land_command(mav)
                rth_to_land = False

        time.sleep(1 / SEND_HZ)





# ───── CLI制御ループ ─────
def control_loop():
    """キーボード操作によるCLI制御"""
    global running, target, takeoff_point, initial_target_set, rth_to_land, takeoff_sent, saved_position

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    tty.setcbreak(fd)

    print("\n" + "="*70)
    print("ドローン制御CLI - キーボードコマンド（NED座標系）")
    print("="*70)
    print("【離陸制御】")
    print("  [t] 離陸（離陸地点自動記録）")
    print("\n【5cm移動（通常）】")
    print("  [w] 南  [s] 北  [a] 東  [d] 西")
    print("  [↑] 上昇  [↓] 下降")
    print("\n【20cm移動（Shift）】")
    print("  [W] 南  [S] 北  [A] 東  [D] 西")
    print("\n【位置記録・移動】")
    print("  [p] 現在位置を記録")
    print("  [o] 記録位置から東へ1m移動")
    print("  [b] 記録位置へ戻る")
    print("  [L] 現在位置を表示")
    print("\n【速度制御（ステップ入力）】")
    print("  [e] 東へ0.2m/s速度指令（長押し）→離すと停止")
    print("\n【RTH】")
    print("  [r] RTH（現在高度で離陸地点直上へ移動して着陸）")
    print("\n【終了】")
    print("  [q] プログラム終了")
    print("="*70)
    print("\n※ NED座標系（X=北、Y=東、Z=下）")
    print("※ GUIDEDモードは目標絶対位置を指定（累積的な移動）")

    try:
        while running:
            key = get_key()
            moved = False
            echo = None

            if key == 'q':
                running = False
                break

            # 離陸（離陸地点自動記録）
            elif key == 't':
                if not guided_mode_active:
                    echo = "⚠ Guidedモードにしてください"
                elif not armed:
                    echo = "⚠ アーム状態にしてください"
                else:
                    # 離陸地点自動記録
                    if origin is None:
                        echo = "⚠ GPS信号未受信"
                    else:
                        with io_lock:
                            takeoff_point = {
                                'lat': origin.lat / 1e7,
                                'lon': origin.lon / 1e7,
                                'alt': origin.relative_alt / 1000
                            }
                        send_takeoff_command(mav, TAKEOFF_ALT)
                        pass
                        takeoff_sent = True
                        echo = f"✓ 離陸地点記録 & 離陸指令送信（{TAKEOFF_ALT:.1f}m）"

            # 5cm移動（NED系：X=北、Y=東、Z=下）
            elif key == 'w':
                if initial_target_set:
                    target['x'] -= 0.05  # 南（-X）
                    moved = True
                    echo = "w 南 -5cm"
                else:
                    echo = "⚠ 離陸完了後に操作可能"
            elif key == 's':
                if initial_target_set:
                    target['x'] += 0.05  # 北（+X）
                    moved = True
                    echo = "s 北 +5cm"
                else:
                    echo = "⚠ 離陸完了後に操作可能"
            elif key == 'a':
                if initial_target_set:
                    target['y'] += 0.05  # 東（+Y）
                    moved = True
                    echo = "a 東 +5cm"
                else:
                    echo = "⚠ 離陸完了後に操作可能"
            elif key == 'd':
                if initial_target_set:
                    target['y'] -= 0.05  # 西（-Y）
                    moved = True
                    echo = "d 西 -5cm"
                else:
                    echo = "⚠ 離陸完了後に操作可能"
            elif key == 'UP':
                if initial_target_set:
                    target['z'] -= 0.05  # 上昇（-Z）
                    moved = True
                    echo = "↑ 上昇 +5cm"
                else:
                    echo = "⚠ 離陸完了後に操作可能"
            elif key == 'DOWN':
                if initial_target_set and target['z'] + 0.05 <= -0.05:
                    target['z'] += 0.05  # 下降（+Z）
                    moved = True
                    echo = "↓ 下降 -5cm"
                elif not initial_target_set:
                    echo = "⚠ 離陸完了後に操作可能"
                else:
                    echo = "⚠ 最低高度制限"

            # 20cm移動（WASD大文字）NED系
            elif key == 'W':
                if initial_target_set:
                    target['x'] -= 0.20  # 南（-X）
                    moved = True
                    echo = "W 南 -20cm"
                else:
                    echo = "⚠ 離陸完了後に操作可能"
            elif key == 'S':
                if initial_target_set:
                    target['x'] += 0.20  # 北（+X）
                    moved = True
                    echo = "S 北 +20cm"
                else:
                    echo = "⚠ 離陸完了後に操作可能"
            elif key == 'A':
                if initial_target_set:
                    target['y'] += 0.20  # 東（+Y）
                    moved = True
                    echo = "A 東 +20cm"
                else:
                    echo = "⚠ 離陸完了後に操作可能"
            elif key == 'D':
                if initial_target_set:
                    target['y'] -= 0.20  # 西（-Y）
                    moved = True
                    echo = "D 西 -20cm"
                else:
                    echo = "⚠ 離陸完了後に操作可能"

            # 位置記録
            elif key == 'p':
                if not initial_target_set:
                    echo = "⚠ 離陸完了後に操作可能"
                else:
                    with io_lock:
                        saved_position = {
                            'x': gps_now['x'],
                            'y': gps_now['y'],
                            'z': gps_now['z']
                        }
                        globals()['saved_position'] = saved_position
                    echo = f"✓ 位置記録: X={saved_position['x']:.2f} Y={saved_position['y']:.2f} Z={saved_position['z']:.2f}({-saved_position['z']:.2f}m高)"

            # 記録位置から東へ1m移動
            elif key == 'o':
                if saved_position is None:
                    echo = "⚠ 位置未記録（先に[p]で記録）"
                elif not initial_target_set:
                    echo = "⚠ 離陸完了後に操作可能"
                else:
                    with io_lock:
                        target['x'] = saved_position['x']
                        target['y'] = saved_position['y'] + 1.0  # 東へ1m（NED系Y+方向）
                        target['z'] = saved_position['z']
                    moved = True
                    echo = f"✓ 記録位置から東へ1m移動 目標: X={target['x']:.2f} Y={target['y']:.2f} Z={target['z']:.2f}"

            # 現在位置を表示（Lキー）
            elif key == 'L':
                with io_lock:
                    echo = f"✓ 現在位置: X={gps_now['x']:.2f} Y={gps_now['y']:.2f} Z={gps_now['z']:.2f} ({-gps_now['z']:.2f}m高)"

            # 速度ステップ入力（eキー：東方向に0.4m/s、2秒間、停止2秒間、その後記録位置へ移動）
            elif key == 'e':
                if not initial_target_set:
                    echo = "⚠ 離陸完了後に操作可能"
                elif saved_position is None:
                    echo = "⚠ 位置未記録（先に[p]で記録）"
                else:
                    echo = f"✓ 速度指令開始: 東へ {STEP_VELOCITY} m/s（2秒間）"
                    sys.stdout.write(f"\x1b[2K\r{echo}\n")
                    sys.stdout.flush()
                    
                    # 2秒間、速度指令を送信
                    step_start = time.time()
                    while time.time() - step_start < 2.0:
                        send_velocity_step(mav, 0.0, STEP_VELOCITY, 0.0)  # 東（+Y）
                        time.sleep(0.05)  # 20Hz
                    
                    # 2秒間、停止指令を送信（急ブレーキ）
                    echo = "✓ 速度指令停止: 急ブレーキ（0m/s）（2秒間）"
                    sys.stdout.write(f"\x1b[2K\r{echo}\n")
                    sys.stdout.flush()
                    stop_start = time.time()
                    while time.time() - stop_start < 2.0:
                        send_velocity_step(mav, 0.0, 0.0, 0.0)
                        time.sleep(0.05)
                    
                    # 記録位置へ戻る
                    echo = "✓ ステップ完了 → 記録位置へ移動開始"
                    sys.stdout.write(f"\x1b[2K\r{echo}\n")
                    sys.stdout.flush()
                    with io_lock:
                        target['x'] = saved_position['x']
                        target['y'] = saved_position['y']
                        target['z'] = saved_position['z']
                    
                    # 目標位置を送信
                    lat, lon, alt = local_xyz_to_gps(target['x'], target['y'], target['z'])
                    send_setpoint(mav, int(lat * 1e7), int(lon * 1e7), alt, YAW_SOUTH)
                    
                    echo = f"✓ 記録位置へ移動中 目標: X={target['x']:.2f} Y={target['y']:.2f} Z={target['z']:.2f}({-target['z']:.2f}m高)"
                    moved = True

            # 記録位置へ戻る
            elif key == 'b':
                if saved_position is None:
                    echo = "⚠ 位置未記録（先に[p]で記録）"
                elif not initial_target_set:
                    echo = "⚠ 離陸完了後に操作可能"
                else:
                    with io_lock:
                        target['x'] = saved_position['x']
                        target['y'] = saved_position['y']
                        target['z'] = saved_position['z']
                    moved = True
                    echo = f"✓ 記録位置へ戻る 目標: X={target['x']:.2f} Y={target['y']:.2f} Z={target['z']:.2f}({-target['z']:.2f}m高)"

            # RTH（NED系）
            elif key == 'r':
                if takeoff_point is None:
                    echo = "⚠ 離陸地点未記録"
                elif not initial_target_set:
                    echo = "⚠ 離陸完了後に操作可能"
                else:
                    with io_lock:
                        current_alt_z = gps_now['z']  # NED系Z（負が上）
                        target['x'] = 0.0
                        target['y'] = 0.0
                        target['z'] = current_alt_z
                        rth_to_land = True
                    moved = True
                    echo = f"✓ RTH実行 現在高度{-current_alt_z:.2f}mで離陸地点直上へ移動→着陸"

            # 目標位置送信
            if moved:
                with io_lock:
                    lat, lon, alt = local_xyz_to_gps(target['x'], target['y'], target['z'])
                    send_setpoint(mav, int(lat * 1e7), int(lon * 1e7), alt, YAW_SOUTH)

            # エコー表示
            if echo:
                sys.stdout.write(f"\x1b[2K\r{echo}\n")
                sys.stdout.flush()
                # ステータス表示（NED系）
                with io_lock:
                    gps_x, gps_y, gps_z = gps_now['x'], gps_now['y'], gps_now['z']
                    tgt_x, tgt_y, tgt_z = target['x'], target['y'], target['z']
                print(f"  現在(NED): X={gps_x:.2f} Y={gps_y:.2f} Z={gps_z:.2f}({-gps_z:.2f}m高) | 目標: X={tgt_x:.2f} Y={tgt_y:.2f} Z={tgt_z:.2f}({-tgt_z:.2f}m高)")

            time.sleep(0.05)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


# ───── メイン関数 ─────
def main():
    global running, mav

    signal.signal(signal.SIGINT, lambda sig, frame: setattr(sys.modules[__name__], "running", False))

    print("="*70)
    print("ドローン制御CLIアプリケーション（NED座標系）")
    print("="*70)

    # MAVLink接続
    mav = connect_mavlink()
    set_msg_rate(mav)

    # 監視・記録スレッド起動

    threading.Thread(target=monitor_vehicle, daemon=True).start()

    # CLI制御ループ
    control_loop()

    print("\n✓ プログラム終了")


if __name__ == "__main__":
    main()
