#!/usr/bin/env python3
"""
square_flight_auto.py - AUTOモードで四角形（矩形）飛行を実行するスクリプト

square_flight_wpnav.py からの派生。
コア変更点: GUIDEDモードで1WPずつ送信する方式から、
AUTOモードで全WPをミッション一括登録する方式に変更。

利点:
  - FC側が WPNAV_RADIUS で自分で到着判定するため、
    ラズパイ側の到着判定ロジック（GPSジッターによるfly-by誤判定）が不要
  - 信頼性が大幅に向上

飛行シーケンス:
  1. MAVLink接続
  2. GPS原点取得
  3. ミッション生成・アップロード（TAKEOFFは含めない。seq0からWAYPOINT）
  4. LOITERモード切替（アーム用）
  5. アーム待機（LOITERモード + プロポでアーム）
  6. 離陸前待機（loiter_after_takeoff_sec）
  7. GUIDEDモード切替 → MAV_CMD_NAV_TAKEOFF を command_long_send
  8. 離陸完了（高度到達）を確認
  9. AUTOモード切替 → ミッション開始
  10. MISSION_CURRENT 監視
  11. 完了検出 → クリーンアップ

エラーハンドリング:
  - JSONファイル不在 → エラー終了
  - ミッションアップロード失敗 → リトライまたはエラー終了
  - AUTOモード移行失敗 → エラー終了
  - 飛行中の異常（ディスアーム、モード変更）→ 安全停止

継承（変更なし）:
  - 全ヘルパー関数、connect、モニター/レコードスレッド、CSV保存、cleanup、main
  - パラメータファイルは square_params.json を共用
"""

import sys
import time
import math
import json
import csv
import datetime
import threading
import signal
from pathlib import Path
from pymavlink import mavutil
import pytz


# =====================================================================
#  ヘルパー関数（モジュールレベル）
# =====================================================================

_METERS_PER_DEG_LAT = 111319.5


def gps_to_local_xyz(lat, lon, alt, ref_lat, ref_lon, ref_alt):
    cos_lat = math.cos(math.radians(ref_lat))
    dx = (lon - ref_lon) * _METERS_PER_DEG_LAT * cos_lat
    dy = (lat - ref_lat) * _METERS_PER_DEG_LAT
    dz = alt - ref_alt
    return dx, dy, dz


def local_xyz_to_gps(x, y, z, ref_lat, ref_lon, ref_alt):
    cos_lat = math.cos(math.radians(ref_lat))
    lat = ref_lat + y / _METERS_PER_DEG_LAT
    lon = ref_lon + x / (_METERS_PER_DEG_LAT * cos_lat)
    alt = ref_alt + z
    return lat, lon, alt


def generate_square_vertices(half_width, half_height, direction):
    V0 = (+half_width, +half_height)
    V1 = (+half_width, -half_height)
    V2 = (-half_width, -half_height)
    V3 = (-half_width, +half_height)
    if direction == "CCW":
        return [V0, V1, V2, V3]
    else:
        return [V0, V3, V2, V1]


# =====================================================================
#  パラメータ読み込み・バリデーション
# =====================================================================

def _validate_params(params):
    center = params["center"]
    lat = center["latitude"]
    lon = center["longitude"]
    if not (-90.0 <= lat <= 90.0):
        raise ValueError(f"center.latitude が範囲外: {lat}（-90.0 〜 90.0）")
    if not (-180.0 <= lon <= 180.0):
        raise ValueError(f"center.longitude が範囲外: {lon}（-180.0 〜 180.0）")
    hw = params["half_width_m"]
    if hw <= 0.0:
        raise ValueError(f"half_width_m は正の値である必要があります: {hw}")
    hh = params["half_height_m"]
    if hh <= 0.0:
        raise ValueError(f"half_height_m は正の値である必要があります: {hh}")
    speed = params["speed_m_s"]
    if speed <= 0.0:
        raise ValueError(f"speed_m_s は正の値である必要があります: {speed}")
    altitude = params["altitude_m"]
    if altitude < 0.5:
        raise ValueError(f"altitude_m が小さすぎます: {altitude}（>= 0.5）")
    stop_sec = params["stop_at_vertex_sec"]
    if stop_sec < 0.0:
        raise ValueError(f"stop_at_vertex_sec が負の値です: {stop_sec}（>= 0）")
    num_laps = params["num_laps"]
    if num_laps < 1:
        raise ValueError(f"num_laps が小さすぎます: {num_laps}（>= 1）")
    direction = params["direction"]
    if direction not in ("CW", "CCW"):
        raise ValueError(f"direction が不正: {direction}")
    yaw_mode = params["yaw_mode"]
    if yaw_mode not in ("fixed", "edge", "center"):
        raise ValueError(f"yaw_mode が不正: {yaw_mode}")
    fixed_yaw = params["fixed_yaw_deg"]
    if not (0.0 <= fixed_yaw <= 360.0):
        raise ValueError(f"fixed_yaw_deg が範囲外: {fixed_yaw}")
    send_hz = params["send_rate_hz"]
    if send_hz < 1:
        raise ValueError(f"send_rate_hz が小さすぎます: {send_hz}（>= 1）")
    takeoff_alt = params["takeoff_alt_m"]
    if takeoff_alt < 0.3:
        raise ValueError(f"takeoff_alt_m が小さすぎます: {takeoff_alt}（>= 0.3）")
    loiter_takeoff = params["loiter_after_takeoff_sec"]
    if loiter_takeoff < 0.0:
        raise ValueError(f"loiter_after_takeoff_sec が負の値です: {loiter_takeoff}")
    loiter_land = params["loiter_before_land_sec"]
    if loiter_land < 0.0:
        raise ValueError(f"loiter_before_land_sec が負の値です: {loiter_land}")


def load_params(path):
    with open(path, 'r') as f:
        params = json.load(f)
    MANDATORY_KEYS = [
        "center", "half_width_m", "half_height_m", "speed_m_s",
        "altitude_m", "stop_at_vertex_sec", "num_laps", "direction",
        "yaw_mode", "fixed_yaw_deg", "takeoff_alt_m", "send_rate_hz",
        "mask", "land_after", "loiter_after_takeoff_sec",
        "loiter_before_land_sec", "wp_timeout", "wpnav_flyby_count",
    ]
    missing = [k for k in MANDATORY_KEYS if k not in params]
    if missing:
        raise ValueError(f"必須キーが不足しています: {', '.join(missing)}")
    center = params["center"]
    for sub_key in ("latitude", "longitude"):
        if sub_key not in center:
            raise ValueError(f"center.{sub_key} が不足しています")
    _validate_params(params)
    return params


# =====================================================================
#  SquareFlightAutoController
# =====================================================================

class SquareFlightAutoController:
    """AUTOモードで四角形飛行を実行するコントローラークラス。"""

    STATE_UPLOAD_MISSION = "UPLOAD_MISSION"
    STATE_WAITING_ARM = "WAITING_ARM"
    STATE_LOITER_ON_GROUND = "LOITER_ON_GROUND"
    STATE_GUIDED_TAKEOFF = "GUIDED_TAKEOFF"
    STATE_AUTO_FLYING = "AUTO_FLYING"
    STATE_COMPLETE = "COMPLETE"

    SERIAL_DEVICE = '/dev/ttyAMA0'
    SERIAL_BAUD = 1000000
    CSV_DIR = Path.home() / "LOGS_Pixhawk6c"

    MODE_NAMES = {
        0: "STABILIZE", 1: "ACRO", 2: "ALT_HOLD",
        3: "AUTO", 4: "GUIDED", 5: "LOITER",
        6: "RTL", 7: "CIRCLE", 9: "LAND",
        11: "DRIFT", 13: "SPORT", 14: "FLIP",
        15: "AUTOTUNE", 16: "POSHOLD", 17: "BRAKE",
        18: "THROW", 19: "AVOID_ADSB", 20: "GUIDED_NOGPS",
        21: "SMART_RTL", 22: "FLOWHOLD", 23: "FOLLOW",
        24: "ZIGZAG", 25: "SYSTEMID", 26: "AUTOROTATE",
        27: "AUTO_RTL",
    }

    def __init__(self, param_path):
        self.params = load_params(param_path)
        print("✓ パラメータ読み込み完了")
        self._print_params()

        self.vertices = generate_square_vertices(
            half_width=self.params["half_width_m"],
            half_height=self.params["half_height_m"],
            direction=self.params["direction"],
        )
        print(f"✓ 矩形頂点生成完了: {len(self.vertices)}点")
        for i, (x, y) in enumerate(self.vertices):
            print(f"    V{i}: ({x:+.3f}, {y:+.3f})m")

        self.master = None
        self._running = False
        self._pause_monitor = False
        self._monitor_thread = None
        self._record_thread = None

        self._io_lock = threading.Lock()
        self._gps_now = {'x': 0.0, 'y': 0.0, 'z': 0.0,
                         'lat': 0.0, 'lon': 0.0, 'alt': 0.0}
        self._target = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self._is_armed = False
        self._is_auto_mode = False
        self._is_guided = False
        self._custom_mode = -1
        self._data_records = []
        self._state = self.STATE_UPLOAD_MISSION
        self._origin = None
        self._takeoff_lat = None
        self._takeoff_lon = None

        self._monitor_loop_count = 0
        self._monitor_hb_count = 0
        self._monitor_pos_count = 0

        self._ref_lat = self.params["center"]["latitude"]
        self._ref_lon = self.params["center"]["longitude"]
        self._ref_alt = 0.0

        self._mission_items = []
        self._mission_count = 0
        self._mission_current_seq = -1
        self._mission_final_seq = -1

        self.CSV_DIR.mkdir(exist_ok=True)

    def _print_params(self):
        p = self.params
        c = p["center"]
        print(f"  中心座標: ({c['latitude']:.7f}, {c['longitude']:.7f})")
        print(f"  矩形サイズ: 幅={p['half_width_m']*2}m x 高さ={p['half_height_m']*2}m")
        print(f"  高度: {p['altitude_m']}m | 速度: {p['speed_m_s']}m/s")
        print(f"  周回数: {p['num_laps']} | 方向: {p['direction']}")
        print(f"  頂点停止: {p['stop_at_vertex_sec']}秒 | 送信レート: {p['send_rate_hz']}Hz")
        print(f"  ヨーモード: {p['yaw_mode']} | land_after: {p['land_after']}")
        print(f"  mode: AUTO（ミッション一括登録方式）")

    def _calc_yaw(self, v_current, v_next):
        yaw_mode = self.params["yaw_mode"]
        if yaw_mode == "fixed":
            return self.params["fixed_yaw_deg"]
        elif yaw_mode == "edge":
            dx = v_next[0] - v_current[0]
            dy = v_next[1] - v_current[1]
            return math.degrees(math.atan2(dx, dy)) % 360
        else:
            dx = -v_current[0]
            dy = -v_current[1]
            return math.degrees(math.atan2(dx, dy)) % 360

    def _generate_mission(self):
        items = []
        altitude = self.params["altitude_m"]
        stop_time = self.params["stop_at_vertex_sec"]
        num_laps = self.params["num_laps"]
        # TAKEOFF はミッションに含めない（ArduCopter は AUTOモードで地上から TAKEOFF 不可）
        # seq 0 から WAYPOINT を開始
        seq = 0
        v0_x, v0_y = self.vertices[0]
        lat_v0, lon_v0, _ = local_xyz_to_gps(
            v0_x, v0_y, altitude,
            self._ref_lat, self._ref_lon, self._ref_alt)
        v0_next = self.vertices[1]
        yaw_v0 = self._calc_yaw((v0_x, v0_y), v0_next)
        items.append({
            'seq': seq,
            'frame': mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            'param1': 0, 'param2': 0, 'param3': 0,
            'param4': yaw_v0,
            'lat': lat_v0, 'lon': lon_v0, 'alt': altitude,
        })
        seq += 1
        for lap in range(num_laps):
            for i in range(4):
                v_target = self.vertices[i]
                v_next = self.vertices[(i + 1) % 4]
                is_last = (lap == num_laps - 1) and (i == 3)
                hold = 0 if is_last else stop_time
                yaw = self._calc_yaw(v_target, v_next)
                lat_wp, lon_wp, _ = local_xyz_to_gps(
                    v_target[0], v_target[1], altitude,
                    self._ref_lat, self._ref_lon, self._ref_alt)
                items.append({
                    'seq': seq,
                    'frame': mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                    'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                    'param1': hold, 'param2': 0, 'param3': 0,
                    'param4': yaw,
                    'lat': lat_wp, 'lon': lon_wp, 'alt': altitude,
                })
                seq += 1
        if self.params["land_after"]:
            loiter_before = self.params["loiter_before_land_sec"]
            items.append({
                'seq': seq,
                'frame': mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                'command': mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
                'param1': loiter_before, 'param2': 0, 'param3': 0,
                'param4': self.params["fixed_yaw_deg"],
                'lat': self._takeoff_lat, 'lon': self._takeoff_lon, 'alt': altitude,
            })
            seq += 1
            items.append({
                'seq': seq,
                'frame': mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
                'command': mavutil.mavlink.MAV_CMD_NAV_LAND,
                'param1': 0, 'param2': 0, 'param3': 0,
                'param4': self.params["fixed_yaw_deg"],
                'lat': self._takeoff_lat, 'lon': self._takeoff_lon, 'alt': 0,
            })
            seq += 1
        return items

    def _print_mission(self, items):
        print(f"\n--- 生成ミッション ({len(items)}アイテム) ---")
        cmd_names = {
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF: "TAKEOFF",
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT: "WAYPOINT",
            mavutil.mavlink.MAV_CMD_NAV_LAND: "LAND",
        }
        for item in items:
            cmd = cmd_names.get(item['command'], f"CMD_{item['command']}")
            print(f"  Seq {item['seq']:2d}: {cmd:10s}"
                  f"  lat={item['lat']:.7f} lon={item['lon']:.7f}"
                  f"  alt={item['alt']:.1f}m"
                  f"  hold={item['param1']:.1f}s yaw={item['param4']:.1f}deg")

    def _upload_mission(self, items, max_retries=3, debug=False):
        count = len(items)
        target_sys = self.master.target_system
        target_comp = self.master.target_component

        # --- MISSION_CLEAR_ALL: 既存ミッションを先にクリア ---
        print("  MISSION_CLEAR_ALL 送信...")
        self.master.mav.mission_clear_all_send(
            target_sys, target_comp,
            mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
        ack = self.master.recv_match(
            type='MISSION_ACK', blocking=True, timeout=5.0)
        if ack is not None:
            ack_dict = ack.to_dict()
            print(f"  MISSION_CLEAR_ALL ACK: type={ack_dict.get('type', -1)}")
        else:
            print("  MISSION_CLEAR_ALL ACK タイムアウト（続行）")

        for attempt in range(max_retries):
            if attempt > 0:
                print(f"  ミッションアップロード リトライ {attempt}/{max_retries}...")
                time.sleep(1.0)
            while True:
                msg = self.master.recv_match(blocking=False)
                if msg is None:
                    break
            self.master.mav.mission_count_send(
                target_sys, target_comp, count,
                mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
            print(f"  MISSION_COUNT({count}) 送信")

            # デバッグ用：FC応答の最初5メッセージtypeをプリント
            _debug_msgs = 0
            _debug_max = 5

            upload_ok = True
            for seq in range(count):
                req = None
                start_wait = time.time()
                while time.time() - start_wait < 5.0:
                    msg = self.master.recv_match(
                        blocking=True, timeout=0.5)
                    if msg is None:
                        continue
                    msg_type = msg.get_type()
                    if debug and _debug_msgs < _debug_max:
                        print(f"  [DEBUG] 受信: type={msg_type}"
                              f" (seq={seq}待ち, {_debug_msgs+1}/{_debug_max})")
                        _debug_msgs += 1
                    if msg_type in ('MISSION_REQUEST', 'MISSION_REQUEST_INT'):
                        msg_dict = msg.to_dict()
                        if msg_dict.get('seq') == seq:
                            req = msg_dict
                            break
                if req is None:
                    print(f"  ✗ MISSION_REQUEST/MISSION_REQUEST_INT(seq={seq})"
                          f" タイムアウト")
                    upload_ok = False
                    break
                item = items[seq]
                self.master.mav.mission_item_int_send(
                    target_sys, target_comp,
                    item['seq'], item['frame'], item['command'],
                    0, 1,
                    item['param1'], item['param2'],
                    item['param3'], item['param4'],
                    int(item['lat'] * 1e7),
                    int(item['lon'] * 1e7),
                    item['alt'],
                    mavutil.mavlink.MAV_MISSION_TYPE_MISSION)
            if not upload_ok:
                continue
            ack = self.master.recv_match(
                type='MISSION_ACK', blocking=True, timeout=5.0)
            if ack is None:
                print("  ✗ MISSION_ACK タイムアウト")
                continue
            ack_dict = ack.to_dict()
            ack_type = ack_dict.get('type', -1)
            if ack_type == mavutil.mavlink.MAV_MISSION_ACCEPTED:
                print(f"  ✓ ミッションアップロード成功 ({count}アイテム)")
                return True
            else:
                print(f"  ✗ MISSION_ACK 失敗: type={ack_type}")
                continue
        return False

    def connect(self):
        print("\n--- MAVLink接続 ---")
        print(f"  デバイス: {self.SERIAL_DEVICE}, {self.SERIAL_BAUD}bps")
        self.master = mavutil.mavlink_connection(
            self.SERIAL_DEVICE, baud=self.SERIAL_BAUD, rtscts=True)
        print("  ハートビート待機中...")
        hb = self.master.wait_heartbeat(timeout=10)
        if hb is None:
            print("✗ ハートビート受信タイムアウト")
            return False
        print(f"✓ MAVLink接続完了 (sys={hb.autopilot}, ver={hb.mavlink_version})")
        for mid in (24, 33, 30, 42):
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0, mid, 200000, 0, 0, 0, 0, 0)
        print("✓ メッセージレート設定完了")
        print(f"[INFO] target_system={self.master.target_system}, "
              f"target_component={self.master.target_component}")
        print("\n--- GPS原点取得 ---")
        print("  GLOBAL_POSITION_INT 待機中...")
        start = time.time()
        while self._origin is None:
            if time.time() - start > 30.0:
                print("✗ GPS原点取得タイムアウト（30秒）")
                return False
            msg = self.master.recv_match(
                type='GLOBAL_POSITION_INT', blocking=True, timeout=0.5)
            if msg is None:
                continue
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            if lat != 0.0 and lon != 0.0:
                self._origin = msg
                self._takeoff_lat = lat
                self._takeoff_lon = lon
                print(f"✓ 原点設定 lat={lat:.7f}, lon={lon:.7f},"
                      f" rel_alt={msg.relative_alt / 1000.0:.2f}m")
        self._running = True
        self._monitor_thread = threading.Thread(
            target=self._monitor_loop, daemon=True, name="monitor")
        self._record_thread = threading.Thread(
            target=self._record_loop, daemon=True, name="record")
        self._monitor_thread.start()
        self._record_thread.start()
        print("✓ モニター/レコードスレッド起動")
        return True

    SEVERITY_NAMES = {
        0: "EMERGENCY", 1: "ALERT", 2: "CRITICAL",
        3: "ERROR", 4: "WARNING", 5: "NOTICE",
        6: "INFO", 7: "DEBUG",
    }

    def _process_message(self, msg):
        msg_type = msg.get_type()
        if msg_type == "HEARTBEAT":
            self._monitor_hb_count += 1
            is_auto = (msg.custom_mode == 3)
            is_guided = (msg.custom_mode == 4)
            is_armed = bool(
                msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            with self._io_lock:
                prev_armed = self._is_armed
                self._is_armed = is_armed
                self._is_auto_mode = is_auto
                self._is_guided = is_guided
                self._custom_mode = msg.custom_mode
                current_state = self._state
            if self._monitor_hb_count % 10 == 0:
                mode_str = self.MODE_NAMES.get(
                    msg.custom_mode, str(msg.custom_mode))
                armed_str = "ARMED" if is_armed else "DISARMED"
                print(f"[HEARTBEAT] mode={mode_str}({msg.custom_mode}) {armed_str}")
            if prev_armed and not is_armed:
                print("\n⚠ ディスアーム検出 → 安全停止")
                self._running = False
                return
            if current_state == self.STATE_AUTO_FLYING:
                if not is_auto:
                    print("\n⚠ モード変更検出（AUTO→他モード）→ 安全停止")
                    self._running = False
                    return
            if current_state == self.STATE_GUIDED_TAKEOFF:
                if not is_guided:
                    print("\n⚠ モード変更検出（GUIDED→他モード）→ 安全停止")
                    self._running = False
                    return
        elif msg_type == "STATUSTEXT":
            severity = msg.severity
            if severity <= 4:
                text = msg.text
                if isinstance(text, bytes):
                    text = text.decode('utf-8', errors='replace').rstrip('\x00')
                sev_name = self.SEVERITY_NAMES.get(severity, f"UNKNOWN({severity})")
                print(f"[STATUSTEXT] {sev_name}: {text}")
        elif msg_type == "GLOBAL_POSITION_INT":
            self._monitor_pos_count += 1
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.relative_alt / 1000.0
            if self._monitor_pos_count % 5 == 0:
                print(f"[GPS] lat={lat:.7f} lon={lon:.7f} rel_alt={alt:.2f}m")
            x, y, z = gps_to_local_xyz(
                lat, lon, alt, self._ref_lat, self._ref_lon, self._ref_alt)
            with self._io_lock:
                self._gps_now.update(
                    {'x': x, 'y': y, 'z': z, 'lat': lat, 'lon': lon, 'alt': alt})
        elif msg_type == "MISSION_CURRENT":
            msg_dict = msg.to_dict()
            new_seq = msg_dict.get('seq', -1)
            with self._io_lock:
                if new_seq != self._mission_current_seq:
                    self._mission_current_seq = new_seq
                    print(f"[MISSION_CURRENT] seq={new_seq}"
                          f" (target final={self._mission_final_seq})")

    def _monitor_loop(self):
        send_hz = self.params["send_rate_hz"]
        while self._running:
            if self._pause_monitor:
                time.sleep(0.1)
                continue
            self._monitor_loop_count += 1
            while True:
                msg = self.master.recv_match(blocking=False)
                if msg is None:
                    break
                self._process_message(msg)
            msg = self.master.recv_match(blocking=True, timeout=0.02)
            if msg is not None:
                self._process_message(msg)
                while True:
                    msg = self.master.recv_match(blocking=False)
                    if msg is None:
                        break
                    self._process_message(msg)
            if self._monitor_loop_count % (send_hz * 5) == 0:
                with self._io_lock:
                    current_z = self._gps_now['z']
                    current_seq = self._mission_current_seq
                print(f"[MONITOR] loop={self._monitor_loop_count}"
                      f" hb={self._monitor_hb_count}"
                      f" pos={self._monitor_pos_count}"
                      f" z={current_z:.2f}m"
                      f" mission_seq={current_seq}")
            time.sleep(1.0 / send_hz)

    def _record_loop(self):
        send_hz = self.params["send_rate_hz"]
        while self._running:
            if self._pause_monitor:
                time.sleep(0.1)
                continue
            with self._io_lock:
                gps = self._gps_now.copy()
                tgt = self._target.copy()
            self._data_records.append([
                time.time(),
                gps['x'], gps['y'], gps['z'],
                tgt['x'], tgt['y'], tgt['z'],
            ])
            time.sleep(1.0 / send_hz)

    def _update_target_from_mission(self, seq):
        if seq < 0 or seq >= len(self._mission_items):
            return
        item = self._mission_items[seq]
        tx, ty, _ = gps_to_local_xyz(
            item['lat'], item['lon'], item['alt'],
            self._ref_lat, self._ref_lon, self._ref_alt)
        with self._io_lock:
            self._target.update({'x': tx, 'y': ty, 'z': item['alt']})

    def wait_for_arm(self):
        print(f"\n--- [{self.STATE_WAITING_ARM}] アーム待機 ---")
        self._state = self.STATE_WAITING_ARM
        timeout = 120.0
        start = time.time()
        while self._running:
            with self._io_lock:
                armed = self._is_armed
            if armed:
                print(f"✓ アーム検出（{time.time() - start:.1f}秒）")
                return True
            if time.time() - start > timeout:
                print(f"⚠ タイムアウト: アーム待機（{timeout}秒）")
                return False
            time.sleep(0.2)
        return False

    def set_auto_mode(self):
        print(f"\n--- AUTOモード切替 ---")
        self.master.set_mode(3)
        start = time.time()
        timeout = 10.0
        while time.time() - start < timeout:
            with self._io_lock:
                is_auto = self._is_auto_mode
            if is_auto:
                print(f"✓ AUTOモード検出（{time.time() - start:.1f}秒）")
                return True
            time.sleep(0.2)
        print(f"✗ AUTOモード切替タイムアウト（{timeout}秒）")
        return False

    def set_guided_mode(self):
        print(f"\n--- GUIDEDモード切替 ---")
        self.master.set_mode(4)
        start = time.time()
        timeout = 10.0
        while time.time() - start < timeout:
            with self._io_lock:
                is_guided = self._is_guided
            if is_guided:
                print(f"✓ GUIDEDモード検出（{time.time() - start:.1f}秒）")
                return True
            time.sleep(0.2)
        print(f"✗ GUIDEDモード切替タイムアウト（{timeout}秒）")
        return False

    def _send_takeoff(self, alt_m):
        """離陸指令を送信する（MAV_CMD_NAV_TAKEOFF）。"""
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,          # confirmation
            0, 0, 0, 0, 0, 0,
            alt_m,      # param7: 離陸高度
        )

    def guided_takeoff(self):
        """
        GUIDEDモードで離陸シーケンスを実行する。

        1. loiter_after_takeoff_sec 秒の安定待機
        2. MAV_CMD_NAV_TAKEOFF 送信（takeoff_alt_m）
        3. 高度が takeoff_alt_m * 0.9 に達するまで待機
        4. 上昇トレンド検出（5秒経過後、3回連続上昇で成功とみなす）

        Returns:
            bool: 離陸成功時 True（タイムアウト: 30秒）
        """
        print(f"\n--- [{self.STATE_GUIDED_TAKEOFF}] GUIDED離陸 ---")
        self._state = self.STATE_GUIDED_TAKEOFF

        takeoff_alt = self.params["takeoff_alt_m"]
        loiter_sec = self.params["loiter_after_takeoff_sec"]

        # 離陸前の安定待機
        if loiter_sec > 0:
            print(f"  離陸前待機 {loiter_sec}秒...")
            for _ in range(int(loiter_sec)):
                if not self._running:
                    return False
                time.sleep(1.0)
            print("✓ 離陸前待機完了")

        # 離陸指令送信
        self._send_takeoff(takeoff_alt)
        print(f"✓ 離陸指令送信（{takeoff_alt}m）")

        # 離陸高度到達待機（タイムアウト: 30秒）
        timeout = 30.0
        start = time.time()
        last_retry = 0.0
        last_reported = -1
        prev_z = 0.0
        rising_count = 0

        while self._running:
            with self._io_lock:
                current_z = self._gps_now['z']

            elapsed = time.time() - start
            if int(elapsed) != last_reported:
                last_reported = int(elapsed)
                print(
                    f"[TAKEOFF] 待機中..."
                    f" {elapsed:.0f}s, z={current_z:.2f}m,"
                    f" target={takeoff_alt * 0.9:.2f}m"
                )

            if current_z >= takeoff_alt * 0.9:
                print(f"✓ 離陸高度到達: {current_z:.2f}m")
                return True

            if elapsed > 5.0:
                if current_z > prev_z + 0.05:
                    rising_count += 1
                    if rising_count >= 3:
                        print(f"✓ 離陸上昇トレンド検出: z={current_z:.2f}m")
                        return True
                else:
                    rising_count = 0
            prev_z = current_z

            if time.time() - start > timeout:
                print(
                    f"⚠ タイムアウト: 離陸高度到達"
                    f"（{timeout}秒, 現在高度={current_z:.2f}m）"
                )
                return False

            if time.time() - last_retry >= 3.0:
                self._send_takeoff(takeoff_alt)
                last_retry = time.time()

            time.sleep(0.1)

        return False

    def set_loiter_mode(self):
        print(f"\n--- LOITERモード切替 ---")
        self.master.set_mode(5)
        start = time.time()
        timeout = 10.0
        while time.time() - start < timeout:
            with self._io_lock:
                mode = self._custom_mode
            if mode == 5:
                print(f"✓ LOITERモード検出（{time.time() - start:.1f}秒）")
                return True
            time.sleep(0.2)
        print(f"✗ LOITERモード切替タイムアウト（{timeout}秒）")
        return False

    def monitor_auto_flight(self):
        land_after = self.params["land_after"]
        final_seq = self._mission_final_seq
        stop_time = self.params["stop_at_vertex_sec"]
        print(f"\n--- [{self.STATE_AUTO_FLYING}] AUTO飛行監視"
              f"（全{self._mission_count}WP, 最終seq={final_seq}）---")
        self._state = self.STATE_AUTO_FLYING
        flight_start = time.time()
        final_reached_time = None
        last_seq_report = -1
        while self._running:
            with self._io_lock:
                current_seq = self._mission_current_seq
                is_armed = self._is_armed
            if current_seq != last_seq_report:
                elapsed = time.time() - flight_start
                print(f"  [MISSION] seq {current_seq}/{final_seq} (elapsed={elapsed:.1f}s)")
                last_seq_report = current_seq
                self._update_target_from_mission(current_seq)
            if current_seq >= final_seq:
                if final_reached_time is None:
                    final_reached_time = time.time()
                    print(f"  ✓ 最終seq到達 ({final_seq})")
                if land_after:
                    if not is_armed:
                        print(f"✓ 飛行完了+ディスアーム検出"
                              f"（{time.time() - flight_start:.1f}秒）")
                        self._state = self.STATE_COMPLETE
                        return True
                    if time.time() - final_reached_time > 60.0:
                        print("⚠ 最終seq到達から60秒経過 → 完了とみなす")
                        self._state = self.STATE_COMPLETE
                        return True
                else:
                    if time.time() - final_reached_time >= stop_time + 3.0:
                        print(f"✓ 飛行完了（着陸なし）"
                              f"（{time.time() - flight_start:.1f}秒）")
                        self._state = self.STATE_COMPLETE
                        return True
            total_edge = sum(
                math.sqrt(
                    (self.vertices[i][0] - self.vertices[(i+1)%4][0])**2
                    + (self.vertices[i][1] - self.vertices[(i+1)%4][1])**2
                ) for i in range(4))
            num_laps = self.params["num_laps"]
            est = total_edge / self.params["speed_m_s"] * num_laps + stop_time * 4 * num_laps + 60.0
            overall = max(est * 2, 120.0)
            if time.time() - flight_start > overall:
                print(f"⚠ 飛行全体タイムアウト（{overall:.0f}秒）→ 完了とみなす")
                self._state = self.STATE_COMPLETE
                return True
            time.sleep(0.5)
        return False

    def run(self):
        print("=" * 60)
        print("  ArduPilot 四角形飛行制御 - Square Flight (AUTO)")
        print("=" * 60)
        try:
            if not self.connect():
                print("\n✗ 接続失敗")
                return

            # 1. ミッション生成・アップロード（TAKEOFFなし）
            print(f"\n--- [{self.STATE_UPLOAD_MISSION}] ミッション生成・アップロード ---")
            self._state = self.STATE_UPLOAD_MISSION
            self._mission_items = self._generate_mission()
            self._mission_count = len(self._mission_items)
            self._mission_final_seq = self._mission_count - 1
            self._print_mission(self._mission_items)
            self._pause_monitor = True
            time.sleep(0.2)
            ok = self._upload_mission(self._mission_items)
            self._pause_monitor = False
            if not ok:
                print("\n✗ ミッションアップロード失敗")
                return

            # 2. LOITERモード切替
            print(f"\n--- [{self.STATE_LOITER_ON_GROUND}] LOITERモード切替 ---")
            self._state = self.STATE_LOITER_ON_GROUND
            if not self.set_loiter_mode():
                print("\n✗ LOITERモード切替失敗")
                return

            # 3. アーム待機
            if not self.wait_for_arm():
                print("\n✗ アーム待機失敗")
                return

            # 4. GUIDEDモード切替 → 離陸
            print("\n--- アーム検出 → GUIDEDモード切替 → 離陸 ---")
            if not self.set_guided_mode():
                print("\n✗ GUIDEDモード切替失敗")
                return
            if not self.guided_takeoff():
                print("\n✗ 離陸失敗")
                return

            # 5. AUTOモード切替 → ミッション開始
            print("\n--- 離陸完了 → AUTOモード切替 → ミッション開始 ---")
            if not self.set_auto_mode():
                print("\n✗ AUTOモード切替失敗（ミッション開始）")
                return

            # 6. MISSION_CURRENT 監視
            if not self.monitor_auto_flight():
                print("\n✗ AUTO飛行中断")
                return

            print(f"\n✓ [{self.STATE_COMPLETE}] 全シーケンス完了")
        except Exception as e:
            print(f"\n✗ 予期せぬエラー: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.cleanup()

    def cleanup(self):
        print("\n--- クリーンアップ ---")
        self._running = False
        for thread, name in [
            (self._monitor_thread, "monitor"),
            (self._record_thread, "record"),
        ]:
            if thread and thread.is_alive():
                thread.join(timeout=3.0)
                if thread.is_alive():
                    print(f"⚠ {name}スレッド終了タイムアウト")
                else:
                    print(f"✓ {name}スレッド終了")
        self._save_csv()
        print("✓ プログラム終了")

    def _save_csv(self):
        if not self._data_records:
            print("⚠ 記録データなし")
            return
        now = datetime.datetime.now(
            pytz.timezone("Asia/Tokyo")).strftime("%Y%m%d_%H%M%S")
        path = self.CSV_DIR / f"{now}_square_auto.csv"
        with open(path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'Time', 'GPS_X', 'GPS_Y', 'GPS_Z',
                'Target_X', 'Target_Y', 'Target_Z'])
            writer.writerows(self._data_records)
        print(f"✓ CSV保存完了: {path}（{len(self._data_records)}行）")


# =====================================================================
#  エントリポイント
# =====================================================================

def main():
    if len(sys.argv) > 1:
        param_path = Path(sys.argv[1])
    else:
        param_path = Path(__file__).parent / "square_params.json"
    if not param_path.exists():
        print(f"✗ パラメータファイルが見つかりません: {param_path}")
        print("  使用法: python square_flight_auto.py [params.json]")
        sys.exit(1)
    print(f"パラメータファイル: {param_path}")
    try:
        controller = SquareFlightAutoController(str(param_path))
    except (FileNotFoundError, json.JSONDecodeError, ValueError) as e:
        print(f"✗ パラメータ読み込みエラー: {e}")
        sys.exit(1)
    def handle_interrupt(signum, frame):
        print(f"\n⚠ 割り込み信号({signum})検出 → 安全停止")
        controller._running = False
    signal.signal(signal.SIGINT, handle_interrupt)
    signal.signal(signal.SIGTERM, handle_interrupt)
    controller.run()


if __name__ == "__main__":
    main()
