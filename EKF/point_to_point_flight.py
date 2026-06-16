#!/usr/bin/env python3
"""
point_to_point_flight.py - Guidedモードで2地点間を直線移動するスクリプト

zigzag_flight.py のアーキテクチャを踏襲し、
スタート地点 → 目標地点 → スタート地点帰還 → 着陸 のシーケンスを実行する。

特徴:
  - 地点間の補間は行わず、目的地のセットポイントを直接送信（即時移動）
  - スタート地点で指定秒数ホバリング後、目標地点へ移動
  - 目標地点で指定秒数ホバリング後、スタート地点に戻り着陸

JSONパラメータファイルから設定を読み込む。
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

# 緯度経度→メートル変換に使用するWGS84近似定数
_METERS_PER_DEG_LAT = 111319.5  # [m/deg]（赤道における1度あたりの距離）


def gps_to_local_xyz(lat, lon, alt, ref_lat, ref_lon, ref_alt):
    """
    GPS座標を基準点からのローカル直交座標に変換する。

    Args:
        lat, lon, alt: 変換元のGPS座標 [deg, deg, m]
        ref_lat, ref_lon, ref_alt: 基準点のGPS座標 [deg, deg, m]

    Returns:
        (dx, dy, dz): 東方向、北方向、上方向の距離 [m]
    """
    cos_lat = math.cos(math.radians(ref_lat))
    dx = (lon - ref_lon) * _METERS_PER_DEG_LAT * cos_lat
    dy = (lat - ref_lat) * _METERS_PER_DEG_LAT
    dz = alt - ref_alt
    return dx, dy, dz


def local_xyz_to_gps(x, y, z, ref_lat, ref_lon, ref_alt):
    """
    ローカル直交座標をGPS座標に変換する。

    Args:
        x, y, z: 東方向、北方向、上方向の距離 [m]
        ref_lat, ref_lon, ref_alt: 基準点のGPS座標 [deg, deg, m]

    Returns:
        (lat, lon, alt): GPS座標 [deg, deg, m]
    """
    cos_lat = math.cos(math.radians(ref_lat))
    lat = ref_lat + y / _METERS_PER_DEG_LAT
    lon = ref_lon + x / (_METERS_PER_DEG_LAT * cos_lat)
    alt = ref_alt + z
    return lat, lon, alt


# =====================================================================
#  パラメータ読み込み・バリデーション
# =====================================================================

def _validate_params(params):
    """
    パラメータのバリデーションを行う。

    Raises:
        ValueError: パラメータが制約を満たさない場合
    """
    # start_position のバリデーション
    sp = params.get("start_position", {})
    lat = sp.get("latitude", 0)
    lon = sp.get("longitude", 0)
    if not (-90.0 <= lat <= 90.0):
        raise ValueError(
            f"start_position.latitude が範囲外: {lat}（-90.0 〜 90.0）")
    if not (-180.0 <= lon <= 180.0):
        raise ValueError(
            f"start_position.longitude が範囲外: {lon}（-180.0 〜 180.0）")

    # altitude_m のバリデーション
    altitude = params.get("altitude_m", 1.0)
    if altitude < 0.5:
        raise ValueError(
            f"altitude_m が小さすぎます: {altitude}（>= 0.5）")


    # loiter_at_start_sec のバリデーション
    loiter_start = params.get("loiter_at_start_sec", 4.0)
    if loiter_start < 0.0:
        raise ValueError(
            f"loiter_at_start_sec が負の値です: {loiter_start}（>= 0）")

    # loiter_at_target_sec のバリデーション
    loiter_target = params.get("loiter_at_target_sec", 5.0)
    if loiter_target < 0.0:
        raise ValueError(
            f"loiter_at_target_sec が負の値です: {loiter_target}（>= 0）")

    # send_rate_hz の最低限チェック
    send_hz = params.get("send_rate_hz", 10)
    if send_hz < 1:
        raise ValueError(
            f"send_rate_hz が小さすぎます: {send_hz}（>= 1）")

    # fixed_yaw_deg のバリデーション
    fixed_yaw = params.get("fixed_yaw_deg", 180.0)
    if not (0.0 <= fixed_yaw <= 360.0):
        raise ValueError(
            f"fixed_yaw_deg が範囲外: {fixed_yaw}（0.0 〜 360.0）")


def load_params(path):
    """
    JSONパラメータファイルを読み込み、バリデーションを行う。

    Args:
        path: JSONファイルのパス

    Returns:
        バリデーション済みのパラメータ辞書

    Raises:
        FileNotFoundError: ファイルが存在しない場合
        ValueError: パラメータが制約を満たさない場合
    """
    with open(path, 'r') as f:
        params = json.load(f)
    _validate_params(params)

    # デフォルト値の補完
    params.setdefault("start_position", {"latitude": 0.0, "longitude": 0.0})
    params.setdefault("altitude_m", 1.0)
    params.setdefault("target_offset_north_m", -0.6)
    params.setdefault("target_offset_east_m", 0.0)
    params.setdefault("loiter_at_start_sec", 4.0)
    params.setdefault("loiter_at_target_sec", 5.0)
    params.setdefault("takeoff_alt_m", 0.5)
    params.setdefault("loiter_after_takeoff_sec", 3.0)
    params.setdefault("loiter_before_land_sec", 3.0)
    params.setdefault("send_rate_hz", 10)
    params.setdefault("fixed_yaw_deg", 180.0)
    params.setdefault("mask", "0x09F8")
    params.setdefault("land_after", True)

    return params


# =====================================================================
#  PointToPointFlightController - メインコントローラークラス
# =====================================================================

class PointToPointFlightController:
    """
    Guidedモードで2地点間の直線移動を実行するコントローラークラス。

    内部状態遷移:
        WAITING_ARM → TAKEOFF → MOVE_TO_START → LOITER_AT_START
        → MOVE_TO_TARGET → LOITER_AT_TARGET → RETURN_TO_START → LANDING → COMPLETE

    スレッド構成（zigzag_flight.py のパターンを踏襲）:
        - メインスレッド: 飛行シーケンス制御
        - モニタースレッド: HEARTBEAT/GPS監視、ディスアーム検出
        - レコードスレッド: CSVデータ記録
    """

    # ── 状態定数 ──
    STATE_WAITING_ARM = "WAITING_ARM"
    STATE_TAKEOFF = "TAKEOFF"
    STATE_MOVE_TO_START = "MOVE_TO_START"
    STATE_LOITER_AT_START = "LOITER_AT_START"
    STATE_MOVE_TO_TARGET = "MOVE_TO_TARGET"
    STATE_LOITER_AT_TARGET = "LOITER_AT_TARGET"
    STATE_RETURN_TO_START = "RETURN_TO_START"
    STATE_LANDING = "LANDING"
    STATE_COMPLETE = "COMPLETE"

    # シリアル接続設定
    SERIAL_DEVICE = '/dev/ttyAMA0'
    SERIAL_BAUD = 1000000

    # CSV保存ディレクトリ
    CSV_DIR = Path.home() / "LOGS_Pixhawk6c"

    # ── モード名マッピング ──
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
        """
        コンストラクタ。
        JSONパラメータ読み込み → バリデーション → 目標地点計算。

        Args:
            param_path: JSONパラメータファイルのパス
        """
        # ── パラメータ読み込みと検証 ──
        self.params = load_params(param_path)
        print("✓ パラメータ読み込み完了")
        self._print_params()

        # ── MAVLink接続オブジェクト ──
        self.master = None

        # ── 実行制御フラグ ──
        self._running = False

        # ── スレッド ──
        self._monitor_thread = None
        self._record_thread = None
        self._altitude_debug_thread = None

        # ── 共有状態（スレッドセーフ、_io_lock で保護）──
        self._io_lock = threading.Lock()
        self._gps_now = {'x': 0.0, 'y': 0.0, 'z': 0.0,
                         'lat': 0.0, 'lon': 0.0, 'alt': 0.0}
        self._target = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self._is_armed = False
        self._is_guided = False
        self._is_land_mode = False
        self._data_records = []
        self._state = self.STATE_WAITING_ARM
        self._origin = None
        self._takeoff_lat = None
        self._takeoff_lon = None

        # ── デバッグ用カウンタ ──
        self._monitor_loop_count = 0
        self._monitor_hb_count = 0
        self._monitor_pos_count = 0

        # ── 基準点: スタート地点──
        self._ref_lat = self.params["start_position"]["latitude"]
        self._ref_lon = self.params["start_position"]["longitude"]
        self._ref_alt = 0.0

        # ── 目標地点のローカル座標とGPS座標を計算 ──
        offset_n = self.params["target_offset_north_m"]
        offset_e = self.params["target_offset_east_m"]
        altitude = self.params["altitude_m"]
        self._target_local_x = offset_e    # 東方向
        self._target_local_y = offset_n    # 北方向（南は負）
        (self._target_lat, self._target_lon, _) = local_xyz_to_gps(
            self._target_local_x, self._target_local_y, altitude,
            self._ref_lat, self._ref_lon, self._ref_alt,
        )

        # ── スタート地点のGPS座標──
        self._start_lat = self._ref_lat
        self._start_lon = self._ref_lon

        # ── 送信用マスク ──
        self._mask = int(self.params["mask"], 16)

        # ── CSV保存先ディレクトリ作成 ──
        self.CSV_DIR.mkdir(exist_ok=True)

    # ──────────────────────────────────────────
    #  表示
    # ──────────────────────────────────────────

    def _print_params(self):
        """読み込んだパラメータの概要を表示する。"""
        p = self.params
        c = p["start_position"]
        print(f"  スタート地点: ({c['latitude']:.7f}, {c['longitude']:.7f})")
        print(f"  高度: {p['altitude_m']}m")
        print(f"  目標オフセット: 北={p['target_offset_north_m']}m,"
              f" 東={p['target_offset_east_m']}m")
        print(f"  スタート待機: {p['loiter_at_start_sec']}秒"
              f" | 目標待機: {p['loiter_at_target_sec']}秒")
        print(f"  送信レート: {p['send_rate_hz']}Hz"
              f" | ヨー: {p['fixed_yaw_deg']}deg")
        print(f"  land_after: {p['land_after']}")

    # ──────────────────────────────────────────
    #  MAVLink 接続
    # ──────────────────────────────────────────

    def connect(self):
        """
        MAVLink接続を確立し、メッセージレートを設定する。
        接続後、モニタースレッドとレコードスレッドを起動する。

        Returns:
            bool: 接続成功時 True
        """
        print("\n--- MAVLink接続 ---")
        print(f"  デバイス: {self.SERIAL_DEVICE}, {self.SERIAL_BAUD}bps")

        self.master = mavutil.mavlink_connection(
            self.SERIAL_DEVICE,
            baud=self.SERIAL_BAUD,
            rtscts=True
        )

        # ハートビート待機（タイムアウト: 10秒）
        print("  ハートビート待機中...")
        hb = self.master.wait_heartbeat(timeout=10)
        if hb is None:
            print("✗ ハートビート受信タイムアウト")
            return False
        print(f"✓ MAVLink接続完了 (sys={hb.autopilot}, ver={hb.mavlink_version})")

        # メッセージレート設定
        for mid in (24, 33, 30):
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0, mid, 200000, 0, 0, 0, 0, 0,
            )
        print("✓ メッセージレート設定完了")
        print(f"[INFO] target_system={self.master.target_system},"
              f" target_component={self.master.target_component}")

        # スレッド起動
        self._running = True
        self._monitor_thread = threading.Thread(
            target=self._monitor_loop, daemon=True, name="monitor")
        self._record_thread = threading.Thread(
            target=self._record_loop, daemon=True, name="record")
        self._altitude_debug_thread = threading.Thread(
            target=self._altitude_debug_loop, daemon=True, name='alt_debug')
        self._monitor_thread.start()
        self._record_thread.start()
        self._altitude_debug_thread.start()
        print("✓ モニター/レコード/高度デバッグスレッド起動")
        return True

    # ──────────────────────────────────────────
    #  モニタースレッド
    # ──────────────────────────────────────────

    def _process_message(self, msg):
        """
        単一のMAVLinkメッセージを処理する。
        HEARTBEAT → モード/Arm追跡、安全停止判定
        GLOBAL_POSITION_INT → GPS位置更新
        """
        msg_type = msg.get_type()

        if msg_type == "HEARTBEAT":
            self._monitor_hb_count += 1
            is_guided = (msg.custom_mode == 4)
            is_land_mode = (msg.custom_mode == 9)
            is_armed = bool(
                msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

            with self._io_lock:
                prev_armed = self._is_armed
                self._is_armed = is_armed
                self._is_guided = is_guided
                self._is_land_mode = is_land_mode
                current_state = self._state

            if self._monitor_hb_count % 10 == 0:
                mode_str = self.MODE_NAMES.get(msg.custom_mode, str(msg.custom_mode))
                armed_str = "ARMED" if is_armed else "DISARMED"
                print(f"[HEARTBEAT] mode={mode_str}({msg.custom_mode}) {armed_str}")

            # ディスアーム検出 → 安全停止
            if prev_armed and not is_armed:
                print("\n⚠ ディスアーム検出 → 安全停止")
                self._running = False
                return

            # モード変更検出
            if current_state not in (self.STATE_WAITING_ARM, self.STATE_COMPLETE):
                if current_state == self.STATE_LANDING:
                    if not is_land_mode and not is_guided:
                        print("\n⚠ モード変更検出（着陸中にLAND/Guided以外）→ 安全停止")
                        self._running = False
                        return
                elif not is_guided:
                    print("\n⚠ モード変更検出（Guided→他モード）→ 安全停止")
                    self._running = False
                    return

        elif msg_type == "GLOBAL_POSITION_INT":
            self._monitor_pos_count += 1
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.relative_alt / 1000.0

            if self._origin is None:
                self._origin = msg
                self._takeoff_lat = lat
                self._takeoff_lon = lon
                print(f"✓ 原点設定 lat={lat:.7f}, lon={lon:.7f}, alt={alt:.2f}m")

            x, y, z = gps_to_local_xyz(
                lat, lon, alt, self._ref_lat, self._ref_lon, self._ref_alt)
            with self._io_lock:
                self._gps_now.update({
                    'x': x, 'y': y, 'z': z,
                    'lat': lat, 'lon': lon, 'alt': alt,
                })

    def _monitor_loop(self):
        """HEARTBEAT/GPS監視ループ（バッファドレイン方式）。"""
        send_hz = self.params["send_rate_hz"]

        while self._running:
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
                print(f"[MONITOR] loop={self._monitor_loop_count}"
                      f" hb={self._monitor_hb_count}"
                      f" pos={self._monitor_pos_count}"
                      f" z={current_z:.2f}m")

            time.sleep(1.0 / send_hz)

    # ──────────────────────────────────────────
    #  レコードスレッド / 高度デバッグスレッド
    # ──────────────────────────────────────────

    def _record_loop(self):
        """CSVデータ記録ループ。"""
        send_hz = self.params["send_rate_hz"]
        while self._running:
            with self._io_lock:
                gps = self._gps_now.copy()
                tgt = self._target.copy()
            self._data_records.append([
                time.time(),
                gps['x'], gps['y'], gps['z'],
                tgt['x'], tgt['y'], tgt['z'],
            ])
            time.sleep(1.0 / send_hz)

    def _altitude_debug_loop(self):
        """高度デバッグループ（2Hz）。"""
        while self._running:
            with self._io_lock:
                actual_z = self._gps_now['z']
                target_z = self._target['z']
            diff = actual_z - target_z
            sign = '+' if diff >= 0 else ''
            print(f"[ALT_DEBUG] actual={actual_z:.2f}m"
                  f" target={target_z:.2f}m diff={sign}{diff:.2f}m")
            time.sleep(0.5)

    # ──────────────────────────────────────────
    #  セットポイント送信
    # ──────────────────────────────────────────

    def _send_setpoint(self, lat_deg, lon_deg, alt_m, yaw_deg):
        """
        位置目標を MAVLink で送信する。

        Args:
            lat_deg: 目標緯度 [deg]
            lon_deg: 目標経度 [deg]
            alt_m: 目標高度（相対高度）[m]
            yaw_deg: 目標ヨー角 [deg]（北=0deg, 東=90deg）
        """
        lat_i = int(lat_deg * 1e7)
        lon_i = int(lon_deg * 1e7)

        self.master.mav.set_position_target_global_int_send(
            0, self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            self._mask,
            lat_i, lon_i, alt_m,
            0, 0, 0, 0, 0, 0,
            math.radians(yaw_deg), 0,
        )

        tx, ty, _ = gps_to_local_xyz(
            lat_deg, lon_deg, alt_m,
            self._ref_lat, self._ref_lon, self._ref_alt,
        )
        with self._io_lock:
            self._target.update({'x': tx, 'y': ty, 'z': alt_m})

    # ──────────────────────────────────────────
    #  離陸 / 着陸指令
    # ──────────────────────────────────────────

    def _send_takeoff(self, alt_m):
        """離陸指令を送信する（MAV_CMD_NAV_TAKEOFF）。"""
        self.master.mav.command_long_send(
            self.master.target_system, self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0, 0, 0, 0, 0, 0, 0, alt_m,
        )

    def _send_land(self):
        """LANDモードに切り替えて着陸する。"""
        self.master.set_mode(9)
        time.sleep(0.5)
        self.master.set_mode(9)

    def _send_land_fallback(self):
        """GUIDEDモードのまま高度0に降下させる。"""
        with self._io_lock:
            lat = self._gps_now.get('lat', 0.0)
            lon = self._gps_now.get('lon', 0.0)
        if lat == 0.0 and lon == 0.0:
            lat, lon = self._ref_lat, self._ref_lon
            print("⚠ GPS未受信のためスタート地点に降下します")
        print(f"  [FALLBACK] GUIDED降下: lat={lat:.7f}, lon={lon:.7f}, alt=0.0m")
        self._send_setpoint(lat, lon, 0.0, 0.0)

    # ──────────────────────────────────────────
    #  飛行シーケンス: wait_for_guided_arm / takeoff
    # ──────────────────────────────────────────

    def wait_for_guided_arm(self):
        """Guidedモード + Arm を待機する。"""
        print(f"\n--- [{self.STATE_WAITING_ARM}] Guided+Arm待機 ---")
        self._state = self.STATE_WAITING_ARM
        timeout = 60.0
        start = time.time()
        while self._running:
            with self._io_lock:
                armed = self._is_armed
                guided = self._is_guided
            if guided and armed:
                print(f"✓ Guidedモード+Arm検出（{time.time() - start:.1f}秒）")
                return True
            if time.time() - start > timeout:
                print(f"⚠ タイムアウト: Guided+Arm待機（{timeout}秒）")
                return False
            time.sleep(0.2)
        return False

    def takeoff(self):
        """離陸シーケンスを実行する。"""
        print(f"\n--- [{self.STATE_TAKEOFF}] 離陸 ---")
        self._state = self.STATE_TAKEOFF
        takeoff_alt = self.params["takeoff_alt_m"]
        loiter_sec = self.params["loiter_after_takeoff_sec"]

        print(f"  離陸前待機 {loiter_sec}秒...")
        for _ in range(int(loiter_sec)):
            if not self._running:
                return False
            time.sleep(1.0)

        self._send_takeoff(takeoff_alt)
        print(f"✓ 離陸指令送信（{takeoff_alt}m）")

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
                print(f"[TAKEOFF] 待機中... {elapsed:.0f}s,"
                      f" z={current_z:.2f}m, target={takeoff_alt * 0.9:.2f}m")

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
                print(f"⚠ タイムアウト: 離陸高度到達"
                      f"（{timeout}秒, 現在高度={current_z:.2f}m）")
                return False

            if time.time() - last_retry >= 3.0:
                self._send_takeoff(takeoff_alt)
                last_retry = time.time()
            time.sleep(0.1)
        return False

    # ──────────────────────────────────────────
    #  飛行シーケンス: move_to_start
    # ──────────────────────────────────────────

    def move_to_start(self):
        """
        スタート地点へ移動し、loiter_at_start_sec 秒ホバリングする。
        離陸後すでにスタート地点付近にいるため、位置合わせ後待機。
        """
        print(f"\n--- [{self.STATE_MOVE_TO_START}] スタート地点へ移動 ---")
        self._state = self.STATE_MOVE_TO_START

        altitude = self.params["altitude_m"]
        yaw = self.params["fixed_yaw_deg"]
        loiter_sec = self.params["loiter_at_start_sec"]
        send_hz = self.params["send_rate_hz"]
        dt = 1.0 / send_hz

        print(f"  目標: lat={self._start_lat:.7f}, lon={self._start_lon:.7f}, alt={altitude}m")
        self._send_setpoint(self._start_lat, self._start_lon, altitude, yaw)

        threshold = 0.3
        timeout = 10.0
        start = time.time()
        last_send = start

        while self._running:
            with self._io_lock:
                x = self._gps_now['x']
                y = self._gps_now['y']
                z = self._gps_now['z']
            dist = math.sqrt(x**2 + y**2)
            if dist < threshold:
                print(f"✓ スタート地点到達: 距離={dist:.2f}m, z={z:.2f}m")
                break
            now = time.time()
            if now - start > timeout:
                print(f"⚠ タイムアウト: スタート地点到達"
                      f"（{timeout}秒, 距離={dist:.2f}m）— 続行します")
                break
            if now - last_send >= 3.0:
                self._send_setpoint(self._start_lat, self._start_lon, altitude, yaw)
                last_send = now
            time.sleep(0.5)

        # ── ホバリング待機 ──
        self._state = self.STATE_LOITER_AT_START
        print(f"\n--- [{self.STATE_LOITER_AT_START}] ホバリング {loiter_sec}秒 ---")
        loiter_start = time.time()
        last_reported = -1
        while time.time() - loiter_start < loiter_sec:
            if not self._running:
                return False
            self._send_setpoint(self._start_lat, self._start_lon, altitude, yaw)
            elapsed = time.time() - loiter_start
            if int(elapsed) != last_reported:
                last_reported = int(elapsed)
                print(f"  スタート待機 {int(elapsed)}/{loiter_sec}秒")
            time.sleep(dt)
        print("✓ スタート地点待機完了")
        return True

    # ──────────────────────────────────────────
    #  飛行シーケンス: move_to_target
    # ──────────────────────────────────────────

    def move_to_target(self):
        """
        目標地点へ即時移動する。
        補間を行わず、目標地点のセットポイントを直接送信し、
        到着後に loiter_at_target_sec 秒ホバリングする。
        """
        print(f"\n--- [{self.STATE_MOVE_TO_TARGET}] 目標地点へ移動 ---")
        self._state = self.STATE_MOVE_TO_TARGET

        altitude = self.params["altitude_m"]
        yaw = self.params["fixed_yaw_deg"]
        loiter_sec = self.params["loiter_at_target_sec"]
        send_hz = self.params["send_rate_hz"]
        dt = 1.0 / send_hz

        print(f"  目標地点: lat={self._target_lat:.7f},"
              f" lon={self._target_lon:.7f}, alt={altitude}m")
        print(f"  ローカル: x={self._target_local_x:+.3f}m,"
              f" y={self._target_local_y:+.3f}m")
        print("  → 即時セットポイント送信（GUIDEDモードで直行）")
        self._send_setpoint(self._target_lat, self._target_lon, altitude, yaw)

        threshold = 0.3
        timeout = 30.0
        start = time.time()
        last_send = start

        while self._running:
            with self._io_lock:
                x = self._gps_now['x']
                y = self._gps_now['y']
                z = self._gps_now['z']
            dist = math.sqrt((x - self._target_local_x)**2 + (y - self._target_local_y)**2)
            if dist < threshold:
                print(f"✓ 目標地点到達: 距離={dist:.2f}m, z={z:.2f}m")
                break
            now = time.time()
            if now - start > timeout:
                print(f"⚠ タイムアウト: 目標地点到達"
                      f"（{timeout}秒, 距離={dist:.2f}m）— 続行します")
                break
            if now - last_send >= 3.0:
                self._send_setpoint(self._target_lat, self._target_lon, altitude, yaw)
                last_send = now
            time.sleep(0.5)

        # ── 目標地点でホバリング待機 ──
        self._state = self.STATE_LOITER_AT_TARGET
        print(f"\n--- [{self.STATE_LOITER_AT_TARGET}] ホバリング {loiter_sec}秒 ---")
        loiter_start = time.time()
        last_reported = -1
        while time.time() - loiter_start < loiter_sec:
            if not self._running:
                return False
            self._send_setpoint(self._target_lat, self._target_lon, altitude, yaw)
            elapsed = time.time() - loiter_start
            if int(elapsed) != last_reported:
                last_reported = int(elapsed)
                print(f"  目標待機 {int(elapsed)}/{loiter_sec}秒")
            time.sleep(dt)
        print("✓ 目標地点待機完了")
        return True

    # ──────────────────────────────────────────
    #  飛行シーケンス: return_to_start
    # ──────────────────────────────────────────

    def return_to_start(self):
        """スタート地点（離陸地点上空）へ戻る。"""
        print(f"\n--- [{self.STATE_RETURN_TO_START}] スタート地点上空へ帰還 ---")
        self._state = self.STATE_RETURN_TO_START

        altitude = self.params["altitude_m"]
        yaw = self.params["fixed_yaw_deg"]

        print(f"  目標: lat={self._start_lat:.7f}, lon={self._start_lon:.7f}, alt={altitude}m")
        print("  → 即時セットポイント送信（GUIDEDモードで直行）")
        self._send_setpoint(self._start_lat, self._start_lon, altitude, yaw)

        threshold = 0.3
        timeout = 30.0
        start = time.time()
        last_send = start

        while self._running:
            with self._io_lock:
                x = self._gps_now['x']
                y = self._gps_now['y']
            dist = math.sqrt(x**2 + y**2)
            if dist < threshold:
                print(f"✓ スタート地点上空に到達: 距離={dist:.2f}m")
                return True
            now = time.time()
            if now - start > timeout:
                print(f"⚠ タイムアウト: スタート地点帰還（{timeout}秒, 距離={dist:.2f}m）")
                return False
            if now - last_send >= 3.0:
                self._send_setpoint(self._start_lat, self._start_lon, altitude, yaw)
                last_send = now
            time.sleep(0.5)
        return False

    # ──────────────────────────────────────────
    #  着陸
    # ──────────────────────────────────────────

    def land(self):
        """着陸シーケンスを実行する。"""
        if not self.params.get("land_after", True):
            print(f"\n--- [{self.STATE_COMPLETE}] land_after=false → 着陸スキップ ---")
            self._state = self.STATE_COMPLETE
            return True

        print(f"\n--- [{self.STATE_LANDING}] 着陸 ---")
        self._state = self.STATE_LANDING

        loiter_sec = self.params["loiter_before_land_sec"]
        print(f"  着陸前待機 {loiter_sec}秒...")
        for _ in range(int(loiter_sec)):
            if not self._running:
                return False
            time.sleep(1.0)

        self._send_land()
        print("✓ 着陸指令送信（set_mode LAND）")

        FALLBACK_TIMEOUT = 10.0
        OVERALL_TIMEOUT = 60.0
        start = time.time()
        land_mode_detected_time = None
        ground_stable_start = None
        prev_z = None
        fallback_active = False
        last_fallback_send = 0.0

        while self._running:
            with self._io_lock:
                armed = self._is_armed
                is_land = self._is_land_mode
                current_z = self._gps_now['z']

            if not armed:
                print("✓ 着陸+ディスアーム完了")
                self._state = self.STATE_COMPLETE
                return True

            if is_land and land_mode_detected_time is None:
                land_mode_detected_time = time.time()
                print(f"  LANDモード検出（z={current_z:.2f}m）")

            elapsed = time.time() - start
            if (not fallback_active and land_mode_detected_time is None
                    and elapsed >= FALLBACK_TIMEOUT):
                print(f"\n⚠ LANDモード未検出（{FALLBACK_TIMEOUT}秒経過）"
                      " → GUIDED降下フォールバック")
                fallback_active = True

            if fallback_active:
                now = time.time()
                if now - last_fallback_send >= 2.0:
                    self._send_land_fallback()
                    last_fallback_send = now
                    if current_z < 0.2:
                        if ground_stable_start is None:
                            ground_stable_start = now
                        elif now - ground_stable_start >= 3.0:
                            print(f"✓ GUIDED降下完了: z={current_z:.2f}m（3秒間低高度安定）")
                            self._state = self.STATE_COMPLETE
                            return True
                    else:
                        ground_stable_start = None
                time.sleep(0.5)
                continue

            # LANDモード正常時の着陸判定
            if current_z < 0.1:
                if prev_z is None:
                    prev_z = current_z
                    ground_stable_start = time.time()
                elif abs(current_z - prev_z) < 0.02:
                    if ground_stable_start is not None:
                        if time.time() - ground_stable_start >= 5.0:
                            print(f"✓ 着陸高度安定検出: z={current_z:.2f}m"
                                  f"（5秒間安定）→ 着陸完了")
                            self._state = self.STATE_COMPLETE
                            return True
                else:
                    ground_stable_start = time.time()
                prev_z = current_z
            else:
                ground_stable_start = None
                prev_z = current_z

            if land_mode_detected_time is not None:
                if time.time() - land_mode_detected_time >= 60.0:
                    print("✓ LANDモード60秒経過 → 着陸完了とみなす")
                    self._state = self.STATE_COMPLETE
                    return True

            if time.time() - start > OVERALL_TIMEOUT:
                print(f"⚠ タイムアウト: 着陸完了待機（{OVERALL_TIMEOUT}秒）")
                self._state = self.STATE_COMPLETE
                return True

            time.sleep(0.5)
        return False

    # ──────────────────────────────────────────
    #  メイン実行 / クリーンアップ
    # ──────────────────────────────────────────

    def run(self):
        """
        メイン飛行シーケンスを実行する。

        シーケンス:
            1. MAVLink接続確立
            2. Guidedモード + Arm 検出
            3. 離陸
            4. スタート地点へ移動 → loiter_at_start_sec 秒ホバリング
            5. 目標地点へ即時移動 → loiter_at_target_sec 秒ホバリング
            6. スタート地点へ帰還 → 着陸
        """
        print("=" * 60)
        print("  ArduPilot ポイント・ツー・ポイント飛行制御")
        print("=" * 60)

        try:
            if not self.connect():
                print("\n✗ 接続失敗")
                return
            if not self.wait_for_guided_arm():
                print("\n✗ Guided+Arm待機失敗")
                return
            if not self.takeoff():
                print("\n✗ 離陸失敗 → 着陸試行")
                self.land()
                return
            if not self.move_to_start():
                print("\n✗ スタート地点移動失敗 → 着陸試行")
                self.land()
                return
            if not self.move_to_target():
                print("\n✗ 目標地点移動失敗 → 着陸試行")
                self.land()
                return
            if self.params.get("land_after", True):
                self.return_to_start()
                # 2秒間スタート位置のsetpointを送信（GUID_TIMEOUT防止）
                _start = time.time()
                while time.time() - _start < 2.0:
                    self._send_setpoint(self._start_lat, self._start_lon,
                                        self.params["altitude_m"],
                                        self.params["fixed_yaw_deg"])
                    time.sleep(0.5)
                self.land()
            print(f"\n✓ [{self.STATE_COMPLETE}] 全シーケンス完了")

        except Exception as e:
            print(f"\n✗ 予期せぬエラー: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.cleanup()

    def cleanup(self):
        """クリーンアップ処理。"""
        print("\n--- クリーンアップ ---")
        self._running = False

        for thread, name in [
            (self._monitor_thread, "monitor"),
            (self._record_thread, "record"),
            (self._altitude_debug_thread, "altitude_debug"),
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
        """
        記録データをCSVファイルに保存する。
        保存先: ~/LOGS_Pixhawk6c/{YYYYMMDD_HHMMSS}_point2point.csv
        """
        if not self._data_records:
            print("⚠ 記録データなし")
            return

        now = datetime.datetime.now(
            pytz.timezone("Asia/Tokyo")
        ).strftime("%Y%m%d_%H%M%S")
        path = self.CSV_DIR / f"{now}_point2point.csv"

        with open(path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'Time', 'GPS_X', 'GPS_Y', 'GPS_Z',
                'Target_X', 'Target_Y', 'Target_Z',
            ])
            writer.writerows(self._data_records)

        print(f"✓ CSV保存完了: {path}（{len(self._data_records)}行）")


# =====================================================================
#  エントリポイント
# =====================================================================

def main():
    """
    スクリプトのエントリポイント。

    使用法:
        python point_to_point_flight.py [パラメータファイルパス]

    引数なしの場合、スクリプトと同じディレクトリの
    point_to_point_params.json を読み込む。
    """
    if len(sys.argv) > 1:
        param_path = Path(sys.argv[1])
    else:
        param_path = Path(__file__).parent / "point_to_point_params.json"

    if not param_path.exists():
        print(f"✗ パラメータファイルが見つかりません: {param_path}")
        print("  使用法: python point_to_point_flight.py [params.json]")
        sys.exit(1)

    print(f"パラメータファイル: {param_path}")

    try:
        controller = PointToPointFlightController(str(param_path))
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
