#!/usr/bin/env python3
"""
circle_flight.py - Guidedモードで円飛行を実行するスクリプト

計画ドキュメント: circle_flight_plan.md に基づく実装。
JSONパラメータファイルから設定を読み込み、
離陸 → 円飛行 → 着陸のシーケンスを自動実行する。

参考コード: /home/taki/Mavlink_raspi/No1/up_to_down3.py
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


def generate_circle_waypoints(center, radius, n, direction,
                              altitude, yaw_mode, fixed_yaw_deg):
    """
    円周上のウェイポイントリストを生成する。

    中心座標を原点とするローカル座標系で等角度間隔の点を計算し、
    GPS座標に変換する。ヨー角は yaw_mode に従って計算する。

    Args:
        center: 中心座標 {'latitude': float, 'longitude': float}
        radius: 円の半径 [m]
        n: ウェイポイント数
        direction: 回転方向 "CW"（時計回り）または "CCW"（反時計回り）
        altitude: 飛行高度 [m]（相対高度）
        yaw_mode: ヨー角モード "tangent" | "center" | "fixed"
        fixed_yaw_deg: yaw_mode="fixed" 時の固定ヨー角 [deg]

    Returns:
        [(lat, lon, alt, yaw_deg), ...] のリスト
    """
    waypoints = []
    ref_lat = center["latitude"]
    ref_lon = center["longitude"]

    for i in range(n):
        # 角度計算（計画ドキュメント 3.1.1節）
        if direction == "CW":
            theta = -2.0 * math.pi * i / n
        else:  # CCW
            theta = 2.0 * math.pi * i / n

        # ローカル座標（中心を原点）
        x = radius * math.cos(theta)  # 東方向
        y = radius * math.sin(theta)  # 北方向

        # GPS座標に変換
        lat_wp, lon_wp, _ = local_xyz_to_gps(
            x, y, 0.0, ref_lat, ref_lon, 0.0
        )

        # ヨー角計算（北=0°, 東=90°）（計画ドキュメント 3.1.3節）
        if yaw_mode == "tangent":
            # 進行方向を向く
            if direction == "CCW":
                yaw = (math.degrees(theta) + 90) % 360
            else:
                yaw = (math.degrees(theta) - 90) % 360
        elif yaw_mode == "center":
            # 円の中心を向く
            yaw = (math.degrees(theta) + 180) % 360
        else:  # fixed
            yaw = fixed_yaw_deg

        waypoints.append((lat_wp, lon_wp, altitude, yaw))

    return waypoints


# =====================================================================
#  パラメータ読み込み・バリデーション（計画ドキュメント 4.2節, 6.2.4節）
# =====================================================================

def _validate_params(params):
    """
    パラメータのバリデーションを行う。

    Raises:
        ValueError: パラメータが制約を満たさない場合
    """
    # center のバリデーション
    center = params.get("center", {})
    lat = center.get("latitude", 0)
    lon = center.get("longitude", 0)
    if not (-90.0 <= lat <= 90.0):
        raise ValueError(
            f"center.latitude が範囲外: {lat}（-90.0 〜 90.0）")
    if not (-180.0 <= lon <= 180.0):
        raise ValueError(
            f"center.longitude が範囲外: {lon}（-180.0 〜 180.0）")

    # radius_m のバリデーション
    radius = params.get("radius_m", 5.0)
    if radius <= 0.0:
        raise ValueError(
            f"radius_m は正の値である必要があります: {radius}")

    # altitude_m のバリデーション
    altitude = params.get("altitude_m", 2.0)
    if altitude < 0.5:
        raise ValueError(
            f"altitude_m が小さすぎます: {altitude}（>= 0.5）")

    # lap_time_sec のバリデーション
    lap_time = params.get("lap_time_sec", 30.0)
    if lap_time < 5.0:
        raise ValueError(
            f"lap_time_sec が小さすぎます: {lap_time}（>= 5.0）")

    # num_laps のバリデーション
    num_laps = params.get("num_laps", 1)
    if num_laps < 1:
        raise ValueError(
            f"num_laps が小さすぎます: {num_laps}（>= 1）")

    # num_waypoints のバリデーション
    n_wp = params.get("num_waypoints", 36)
    if not (8 <= n_wp <= 360):
        raise ValueError(
            f"num_waypoints が範囲外: {n_wp}（8 〜 360）")

    # direction のバリデーション
    direction = params.get("direction", "CW")
    if direction not in ("CW", "CCW"):
        raise ValueError(
            f"direction が不正: {direction}（'CW' または 'CCW'）")

    # yaw_mode のバリデーション
    yaw_mode = params.get("yaw_mode", "tangent")
    if yaw_mode not in ("tangent", "center", "fixed"):
        raise ValueError(
            f"yaw_mode が不正: {yaw_mode}"
            "（'tangent' / 'center' / 'fixed'）")

    # fixed_yaw_deg のバリデーション
    fixed_yaw = params.get("fixed_yaw_deg", 0.0)
    if not (0.0 <= fixed_yaw <= 360.0):
        raise ValueError(
            f"fixed_yaw_deg が範囲外: {fixed_yaw}（0.0 〜 360.0）")

    # send_rate_hz の最低限チェック
    send_hz = params.get("send_rate_hz", 10)
    if send_hz < 1:
        raise ValueError(
            f"send_rate_hz が小さすぎます: {send_hz}（>= 1）")


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
    params.setdefault("radius_m", 5.0)
    params.setdefault("altitude_m", 2.0)
    params.setdefault("lap_time_sec", 30.0)
    params.setdefault("num_laps", 1)
    params.setdefault("num_waypoints", 36)
    params.setdefault("direction", "CW")
    params.setdefault("yaw_mode", "tangent")
    params.setdefault("fixed_yaw_deg", 0.0)
    params.setdefault("takeoff_alt_m", 0.5)
    params.setdefault("send_rate_hz", 10)
    params.setdefault("mask", "0x09F8")
    params.setdefault("land_after", True)
    params.setdefault("loiter_after_takeoff_sec", 3.0)
    params.setdefault("loiter_before_land_sec", 3.0)

    return params


# =====================================================================
#  CircleFlightController - メインコントローラークラス
# =====================================================================

class CircleFlightController:
    """
    Guidedモードで円飛行を実行するコントローラークラス。

    内部状態遷移（計画ドキュメント 3.2節）:
        WAITING_ARM → TAKEOFF → CIRCLE_START → CIRCLE_FLYING
        → CIRCLE_COMPLETE → LANDING → COMPLETE

    スレッド構成（計画ドキュメント 5.2節）:
        - メインスレッド: 飛行シーケンス制御
        - モニタースレッド: HEARTBEAT/GPS監視、ディスアーム検出
        - レコードスレッド: CSVデータ記録
    """

    # ── 状態定数 ──
    STATE_WAITING_ARM = "WAITING_ARM"
    STATE_TAKEOFF = "TAKEOFF"
    STATE_CIRCLE_START = "CIRCLE_START"
    STATE_CIRCLE_FLYING = "CIRCLE_FLYING"
    STATE_CIRCLE_COMPLETE = "CIRCLE_COMPLETE"
    STATE_LANDING = "LANDING"
    STATE_COMPLETE = "COMPLETE"

    # シリアル接続設定（参考コードのパターンを踏襲）
    SERIAL_DEVICE = '/dev/ttyAMA0'
    SERIAL_BAUD = 1000000

    # CSV保存ディレクトリ（参考コードと同様）
    CSV_DIR = Path.home() / "LOGS_Pixhawk6c"

    def __init__(self, param_path):
        """
        コンストラクタ。
        JSONパラメータ読み込み → バリデーション → ウェイポイント生成。

        Args:
            param_path: JSONパラメータファイルのパス
        """
        # ── パラメータ読み込みと検証 ──
        self.params = load_params(param_path)
        print("✓ パラメータ読み込み完了")
        self._print_params()

        # ── ウェイポイント生成 ──
        self.waypoints = generate_circle_waypoints(
            center=self.params["center"],
            radius=self.params["radius_m"],
            n=self.params["num_waypoints"],
            direction=self.params["direction"],
            altitude=self.params["altitude_m"],
            yaw_mode=self.params["yaw_mode"],
            fixed_yaw_deg=self.params["fixed_yaw_deg"],
        )
        print(f"✓ ウェイポイント生成完了: {len(self.waypoints)}点")

        # ── MAVLink接続オブジェクト ──
        self.master = None

        # ── 実行制御フラグ ──
        self._running = False

        # ── スレッド ──
        self._monitor_thread = None
        self._record_thread = None

        # ── 共有状態（スレッドセーフ、_io_lock で保護）──
        self._io_lock = threading.Lock()
        self._gps_now = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self._target = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self._is_armed = False
        self._is_guided = False
        self._data_records = []          # CSV記録用データ
        self._state = self.STATE_WAITING_ARM
        self._origin = None              # 初回GPS位置

        # ── デバッグ用カウンタ ──
        self._monitor_loop_count = 0
        self._monitor_hb_count = 0
        self._monitor_pos_count = 0

        # ── 基準点: 円の中心座標 ──
        self._ref_lat = self.params["center"]["latitude"]
        self._ref_lon = self.params["center"]["longitude"]
        self._ref_alt = 0.0

        # ── 送信用マスク（set_position_target_global_int）──
        self._mask = int(self.params["mask"], 16)

        # ── CSV保存先ディレクトリ作成 ──
        self.CSV_DIR.mkdir(exist_ok=True)

    # ──────────────────────────────────────────
    #  表示
    # ──────────────────────────────────────────

    def _print_params(self):
        """読み込んだパラメータの概要を表示する。"""
        p = self.params
        c = p["center"]
        print(f"  中心座標: ({c['latitude']:.7f}, {c['longitude']:.7f})")
        print(f"  半径: {p['radius_m']}m | 高度: {p['altitude_m']}m")
        print(f"  周回数: {p['num_laps']} | 一周時間: {p['lap_time_sec']}秒")
        print(f"  WP数: {p['num_waypoints']} | 方向: {p['direction']}")
        print(f"  ヨーモード: {p['yaw_mode']} | land_after: {p['land_after']}")

    # ──────────────────────────────────────────
    #  MAVLink 接続（計画ドキュメント 6.1.2節）
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

        # MAVLink接続（参考コードのパターンを踏襲）
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
        print(
            f"✓ MAVLink接続完了"
            f" (sys={hb.autopilot}, ver={hb.mavlink_version})"
        )

        # メッセージレート設定
        #   GPS_RAW_INT=24, GLOBAL_POSITION_INT=33, ATTITUDE=30
        for mid in (24, 33, 30):
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
                0,          # confirmation
                mid,        # param1: message ID
                200000,     # param2: interval [us] = 5Hz
                0, 0, 0, 0, 0,
            )
        print("✓ メッセージレート設定完了")

        # 初期化時追加情報
        print(f"[INFO] target_system={self.master.target_system}, target_component={self.master.target_component}")
        print(f"[INFO] メッセージレート設定: GPS_RAW_INT(24), GLOBAL_POSITION_INT(33), ATTITUDE(30) → 5Hz")

        # スレッド起動（計画ドキュメント 5.2節）
        self._running = True
        self._monitor_thread = threading.Thread(
            target=self._monitor_loop, daemon=True,
            name="monitor"
        )
        self._record_thread = threading.Thread(
            target=self._record_loop, daemon=True,
            name="record"
        )
        self._monitor_thread.start()
        self._record_thread.start()
        print("✓ モニター/レコードスレッド起動")

        return True

    # ──────────────────────────────────────────
    #  モニタースレッド（計画ドキュメント 5.2節, 8.1節）
    # ──────────────────────────────────────────

    def _monitor_loop(self):
        """
        HEARTBEAT/GPS監視ループ。
        - 単一の recv_match() で全メッセージを受信し、get_type() で振り分け。
          (recv_match(type=...) はバッファ内の他タイプを破棄するため、
           HEARTBEAT → GLOBAL_POSITION_INT の順に呼ぶと後者が消失する)
        - HEARTBEAT から Guidedモード / Arm状態を追跡
        - GLOBAL_POSITION_INT から現在GPS位置を更新
        - ディスアーム検出 → _running=False で安全停止
        - 飛行中のモード変更検出 → _running=False で安全停止
        """
        send_hz = self.params["send_rate_hz"]

        # mode_names はループ外で一度だけ定義
        mode_names = {
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

        while self._running:
            # デバッグ: ループカウンタ
            self._monitor_loop_count += 1

            # ── 単一recv_matchで全メッセージを受信し、get_type()で振り分け ──
            #     (type=指定のrecv_matchはバッファ内の非該当メッセージを破棄するため)
            msg = self.master.recv_match(blocking=True, timeout=0.02)
            if msg is not None:
                msg_type = msg.get_type()

                if msg_type == "HEARTBEAT":
                    self._monitor_hb_count += 1
                    is_guided = (msg.custom_mode == 4)
                    is_armed = bool(
                        msg.base_mode
                        & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    )

                    with self._io_lock:
                        prev_armed = self._is_armed
                        self._is_armed = is_armed
                        self._is_guided = is_guided
                        current_state = self._state

                    # HEARTBEAT デバッグ: 約1秒毎にモード情報表示
                    if self._monitor_hb_count % 10 == 0:
                        mode_str = mode_names.get(
                            msg.custom_mode, str(msg.custom_mode)
                        )
                        armed_str = "ARMED" if is_armed else "DISARMED"
                        print(
                            f"[HEARTBEAT] mode={mode_str}({msg.custom_mode})"
                            f" {armed_str}"
                        )

                    # ディスアーム検出 → 安全停止
                    if prev_armed and not is_armed:
                        print("\n⚠ ディスアーム検出 → 安全停止")
                        self._running = False
                        break

                    # モード変更検出（飛行中にGuided以外になった場合）
                    if current_state not in (
                        self.STATE_WAITING_ARM,
                        self.STATE_COMPLETE,
                    ):
                        if not is_guided:
                            print(
                                "\n⚠ モード変更検出"
                                "（Guided→他モード）→ 安全停止"
                            )
                            self._running = False
                            break

                elif msg_type == "GLOBAL_POSITION_INT":
                    self._monitor_pos_count += 1
                    lat = msg.lat / 1e7
                    lon = msg.lon / 1e7
                    alt = msg.relative_alt / 1000.0  # 相対高度 [m]

                    # GPS デバッグ: 約1秒毎に受信ログ
                    if self._monitor_pos_count % 5 == 0:
                        print(
                            f"[GPS] lat={lat:.7f} lon={lon:.7f}"
                            f" rel_alt={alt:.2f}m raw={msg.relative_alt}"
                        )

                    # 原点設定（初回のみ）
                    if self._origin is None:
                        self._origin = msg
                        print(
                            f"✓ 原点設定 lat={lat:.7f},"
                            f" lon={lon:.7f}, alt={alt:.2f}m"
                        )

                    # ローカル座標に変換（中心基準）
                    x, y, z = gps_to_local_xyz(
                        lat, lon, alt,
                        self._ref_lat, self._ref_lon, self._ref_alt
                    )
                    with self._io_lock:
                        self._gps_now.update({'x': x, 'y': y, 'z': z})

            # デバッグ: 約5秒毎にモニターサマリ出力 (send_hz * 5)
            if self._monitor_loop_count % (send_hz * 5) == 0:
                with self._io_lock:
                    current_z = self._gps_now['z']
                print(
                    f"[MONITOR] loop={self._monitor_loop_count}"
                    f" hb={self._monitor_hb_count}"
                    f" pos={self._monitor_pos_count}"
                    f" z={current_z:.2f}m"
                )

            time.sleep(1.0 / send_hz)

    # ──────────────────────────────────────────
    #  レコードスレッド（計画ドキュメント 5.4節）
    # ──────────────────────────────────────────

    def _record_loop(self):
        """
        CSVデータ記録ループ。
        参考コードと同一フォーマットで、send_rate_hz の間隔で記録する。

        CSVフォーマット:
            Time, GPS_X, GPS_Y, GPS_Z, Target_X, Target_Y, Target_Z
        """
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

    # ──────────────────────────────────────────
    #  セットポイント送信（計画ドキュメント 7.1節）
    # ──────────────────────────────────────────

    def _send_setpoint(self, lat_deg, lon_deg, alt_m, yaw_deg):
        """
        位置目標を MAVLink で送信する。

        set_position_target_global_int_send を使用し、
        マスクで位置とヨー角のみを制御対象とする。

        Args:
            lat_deg: 目標緯度 [deg]
            lon_deg: 目標経度 [deg]
            alt_m: 目標高度（相対高度）[m]
            yaw_deg: 目標ヨー角 [deg]（北=0deg, 東=90deg）
        """
        lat_i = int(lat_deg * 1e7)
        lon_i = int(lon_deg * 1e7)

        self.master.mav.set_position_target_global_int_send(
            0,                                      # time_boot_ms
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            self._mask,                             # 制御マスク
            lat_i, lon_i, alt_m,                    # 目標位置
            0, 0, 0,                                # 速度（制御しない）
            0, 0, 0,                                # 加速度（制御しない）
            math.radians(yaw_deg),                  # 目標ヨー角 [rad]
            0,                                      # ヨーレート（制御しない）
        )

        # ログ用に目標位置をローカル座標で更新
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
        """
        離陸指令を送信する（MAV_CMD_NAV_TAKEOFF）。

        Args:
            alt_m: 離陸高度 [m]
        """
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
            0,          # confirmation
            0, 0, 0, 0, 0, 0,
            alt_m,      # param7: 離陸高度
        )

    def _send_land(self):
        """着陸指令を送信する（MAV_CMD_NAV_LAND）。"""
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_LAND,
            0,          # confirmation
            0, 0, 0, 0, 0, 0, 0,
        )

    # ──────────────────────────────────────────
    #  飛行シーケンス（計画ドキュメント 3.2節, 6.1.3〜6.1.7節）
    # ──────────────────────────────────────────

    def wait_for_guided_arm(self):
        """
        Guidedモード + Arm を待機する。

        HEARTBEAT の custom_mode==4（Guided）かつ
        base_mode に SAFETY_ARMED フラグが立つまで待機。

        Returns:
            bool: 検出成功時 True（タイムアウト: 60秒）
        """
        print(
            f"\n--- [{self.STATE_WAITING_ARM}]"
            " Guided+Arm待機 ---"
        )
        self._state = self.STATE_WAITING_ARM

        timeout = 60.0
        start = time.time()

        while self._running:
            with self._io_lock:
                armed = self._is_armed
                guided = self._is_guided

            if guided and armed:
                elapsed = time.time() - start
                print(f"✓ Guidedモード+Arm検出（{elapsed:.1f}秒）")
                return True

            if time.time() - start > timeout:
                print(
                    f"⚠ タイムアウト: Guided+Arm待機（{timeout}秒）"
                )
                return False

            time.sleep(0.2)

        return False

    def takeoff(self):
        """
        離陸シーケンスを実行する。

        1. loiter_after_takeoff_sec 秒の安定待機
        2. MAV_CMD_NAV_TAKEOFF 送信（takeoff_alt_m）
        3. 高度が takeoff_alt_m * 0.9 に達するまで待機

        Returns:
            bool: 離陸成功時 True（タイムアウト: 30秒）
        """
        print(f"\n--- [{self.STATE_TAKEOFF}] 離陸 ---")
        self._state = self.STATE_TAKEOFF

        takeoff_alt = self.params["takeoff_alt_m"]
        loiter_sec = self.params["loiter_after_takeoff_sec"]

        # 離陸前の安定待機
        print(f"  離陸前待機 {loiter_sec}秒...")
        for _ in range(int(loiter_sec)):
            if not self._running:
                return False
            time.sleep(1.0)

        # 離陸指令送信
        self._send_takeoff(takeoff_alt)
        print(f"✓ 離陸指令送信（{takeoff_alt}m）")

        # 離陸高度到達待機（タイムアウト: 30秒）
        timeout = 30.0
        start = time.time()
        last_retry = 0.0  # 直近のTAKEOFF再送時刻
        last_reported = -1  # デバッグ用

        while self._running:
            with self._io_lock:
                current_z = self._gps_now['z']

            # デバッグ: 1秒毎に状態表示
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

            if time.time() - start > timeout:
                print(
                    f"⚠ タイムアウト: 離陸高度到達"
                    f"（{timeout}秒, 現在高度={current_z:.2f}m）"
                )
                return False

            # 3秒間隔でTAKEOFFコマンドを再送（初回が無視された場合のリカバリ）
            if time.time() - last_retry >= 3.0:
                self._send_takeoff(takeoff_alt)
                last_retry = time.time()

            time.sleep(0.1)

        return False

    def move_to_start_position(self):
        """
        円周上の最初のウェイポイント（theta_0）へ移動する。

        - 目標高度 altitude_m に到達するまで待機
        - 目標WPから半径の20%以内に近づくまで待機
        - 到達前に定期的にセットポイントを再送信（3秒間隔）

        Returns:
            bool: 到達成功時 True
        """
        print(
            f"\n--- [{self.STATE_CIRCLE_START}]"
            " 円開始位置へ移動 ---"
        )
        self._state = self.STATE_CIRCLE_START

        wp = self.waypoints[0]
        lat_wp, lon_wp, alt_wp, yaw_wp = wp

        self._send_setpoint(lat_wp, lon_wp, alt_wp, yaw_wp)
        print(
            f"  目標WP[0]: lat={lat_wp:.7f}, lon={lon_wp:.7f},"
            f" alt={alt_wp}m, yaw={yaw_wp:.1f}deg"
        )

        # 目標WPのローカル座標
        wx, wy, _ = gps_to_local_xyz(
            lat_wp, lon_wp, alt_wp,
            self._ref_lat, self._ref_lon, self._ref_alt,
        )

        # 到達閾値: 半径の20%（計画ドキュメント 6.1.5節）
        threshold = self.params["radius_m"] * 0.2
        target_alt = self.params["altitude_m"]

        # 高度到達 + 水平位置到達 の複合待機
        timeout = 90.0
        start = time.time()
        last_send = start
        alt_reached = False

        while self._running:
            with self._io_lock:
                x = self._gps_now['x']
                y = self._gps_now['y']
                z = self._gps_now['z']

            # 高度チェック
            if not alt_reached and z >= target_alt * 0.9:
                alt_reached = True
                print(f"✓ 目標高度到達: {z:.2f}m")

            # 水平位置チェック（高度到達後に評価）
            if alt_reached:
                dist = math.sqrt((x - wx) ** 2 + (y - wy) ** 2)
                if dist < threshold:
                    print(
                        f"✓ 開始位置到達: 距離={dist:.2f}m"
                        f"（閾値={threshold:.2f}m）"
                    )
                    return True

            # 定期的に目標を再送信（3秒間隔）
            now = time.time()
            if now - last_send >= 3.0:
                self._send_setpoint(lat_wp, lon_wp, alt_wp, yaw_wp)
                last_send = now

            # タイムアウト
            if now - start > timeout:
                dist = math.sqrt((x - wx) ** 2 + (y - wy) ** 2)
                print(
                    f"⚠ タイムアウト: 開始位置到達"
                    f"（{timeout}秒, 距離={dist:.2f}m）— 続行します"
                )
                return True  # タイムアウトでも飛行続行

            time.sleep(0.5)

        return False

    def execute_circle_flight(self):
        """
        円飛行のメインループを実行する。

        - num_laps 回ループ
        - 各ラップで全ウェイポイントを順次処理
        - 各ウェイポイント間隔 dt = lap_time_sec / num_waypoints 秒
        - 各ウェイポイントで _send_setpoint() を呼び出す

        Returns:
            bool: 飛行成功時 True
        """
        print(
            f"\n--- [{self.STATE_CIRCLE_FLYING}] 円飛行実行 ---"
        )
        self._state = self.STATE_CIRCLE_FLYING

        n_wp = self.params["num_waypoints"]
        n_laps = self.params["num_laps"]
        dt = self.params["lap_time_sec"] / n_wp
        direction = self.params["direction"]

        estimated_total = self.params["lap_time_sec"] * n_laps
        print(f"  WP数: {n_wp} | 周回数: {n_laps} | 方向: {direction}")
        print(
            f"  送信間隔: {dt:.3f}秒"
            f" | 推定時間: {estimated_total:.1f}秒"
        )

        wp_send_count = 0
        flight_start = time.time()

        for lap in range(n_laps):
            if not self._running:
                print("⚠ 飛行中断")
                return False

            print(f"\n  --- 周回 {lap + 1}/{n_laps} ---")
            lap_start = time.time()

            for i in range(n_wp):
                if not self._running:
                    print("⚠ 飛行中断")
                    return False

                wp = self.waypoints[i]
                lat_wp, lon_wp, alt_wp, yaw_wp = wp

                # セットポイント送信
                self._send_setpoint(lat_wp, lon_wp, alt_wp, yaw_wp)
                wp_send_count += 1

                # 次のウェイポイント送信タイミングまで待機
                # 計画ドキュメント 3.1.2節: t_i = i * dt
                target_time = lap_start + (i + 1) * dt
                sleep_time = target_time - time.time()
                if sleep_time > 0:
                    time.sleep(sleep_time)

            lap_elapsed = time.time() - lap_start
            print(
                f"  ✓ 周回 {lap + 1} 完了"
                f"（{lap_elapsed:.1f}秒/{wp_send_count}送信）"
            )

        total_elapsed = time.time() - flight_start
        print(
            f"\n✓ 全周回完了"
            f"（{total_elapsed:.1f}秒, {wp_send_count}回送信）"
        )
        self._state = self.STATE_CIRCLE_COMPLETE
        return True

    def land(self):
        """
        着陸シーケンスを実行する。

        1. land_after=False の場合は着陸をスキップ
        2. loiter_before_land_sec 秒の安定待機
        3. MAV_CMD_NAV_LAND 送信
        4. ディスアーム検出まで待機（タイムアウト: 60秒）

        Returns:
            bool: 着陸成功時 True
        """
        # land_after=false の場合は着陸スキップ
        if not self.params.get("land_after", True):
            print(
                f"\n--- [{self.STATE_COMPLETE}]"
                " land_after=false → 着陸スキップ ---"
            )
            self._state = self.STATE_COMPLETE
            return True

        print(f"\n--- [{self.STATE_LANDING}] 着陸 ---")
        self._state = self.STATE_LANDING

        loiter_sec = self.params["loiter_before_land_sec"]

        # 着陸前の安定待機
        print(f"  着陸前待機 {loiter_sec}秒...")
        for _ in range(int(loiter_sec)):
            if not self._running:
                return False
            time.sleep(1.0)

        # 着陸指令送信
        self._send_land()
        print("✓ 着陸指令送信")

        # ディスアーム検出待機（タイムアウト: 60秒）
        timeout = 60.0
        start = time.time()

        while self._running:
            with self._io_lock:
                armed = self._is_armed

            if not armed:
                print("✓ 着陸+ディスアーム完了")
                self._state = self.STATE_COMPLETE
                return True

            if time.time() - start > timeout:
                print(
                    f"⚠ タイムアウト: 着陸完了待機（{timeout}秒）"
                )
                self._state = self.STATE_COMPLETE
                return True

            time.sleep(0.5)

        return False

    # ──────────────────────────────────────────
    #  メイン実行 / クリーンアップ（計画ドキュメント 6.1.8〜6.1.9節）
    # ──────────────────────────────────────────

    def run(self):
        """
        メイン飛行シーケンスを実行する。

        シーケンス:
            1. MAVLink接続確立
            2. Guidedモード + Arm 検出
            3. 離陸
            4. 円開始位置へ移動
            5. 円飛行実行
            6. 着陸
        """
        print("=" * 60)
        print("  ArduPilot 円飛行制御 - Circle Flight")
        print("=" * 60)

        try:
            # 1. MAVLink接続
            if not self.connect():
                print("\n✗ 接続失敗")
                return

            # 2. Guided+Arm待機
            if not self.wait_for_guided_arm():
                print("\n✗ Guided+Arm待機失敗")
                return

            # 3. 離陸
            if not self.takeoff():
                print("\n✗ 離陸失敗 → 着陸試行")
                self.land()
                return

            # 4. 円開始位置へ移動
            if not self.move_to_start_position():
                print("\n✗ 開始位置移動失敗 → 着陸試行")
                self.land()
                return

            # 5. 円飛行実行
            if not self.execute_circle_flight():
                print("\n✗ 円飛行中断 → 着陸試行")
                self.land()
                return

            # 6. 着陸
            self.land()
            print(f"\n✓ [{self.STATE_COMPLETE}] 全シーケンス完了")

        except Exception as e:
            print(f"\n✗ 予期せぬエラー: {e}")
            import traceback
            traceback.print_exc()
        finally:
            self.cleanup()

    def cleanup(self):
        """
        クリーンアップ処理。

        1. _running = False（全スレッド停止指示）
        2. モニター/レコードスレッドの終了待機
        3. CSVデータ保存
        """
        print("\n--- クリーンアップ ---")
        self._running = False

        # スレッド終了待機
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

        # CSV保存
        self._save_csv()
        print("✓ プログラム終了")

    # ──────────────────────────────────────────
    #  CSV保存（計画ドキュメント 5.4節）
    # ──────────────────────────────────────────

    def _save_csv(self):
        """
        記録データをCSVファイルに保存する。

        保存先: ~/LOGS_Pixhawk6c/{YYYYMMDD_HHMMSS}_circle.csv
        フォーマット: Time, GPS_X, GPS_Y, GPS_Z, Target_X, Target_Y, Target_Z
        """
        if not self._data_records:
            print("⚠ 記録データなし")
            return

        now = datetime.datetime.now(
            pytz.timezone("Asia/Tokyo")
        ).strftime("%Y%m%d_%H%M%S")
        path = self.CSV_DIR / f"{now}_circle.csv"

        with open(path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'Time', 'GPS_X', 'GPS_Y', 'GPS_Z',
                'Target_X', 'Target_Y', 'Target_Z',
            ])
            writer.writerows(self._data_records)

        print(
            f"✓ CSV保存完了: {path}"
            f"（{len(self._data_records)}行）"
        )


# =====================================================================
#  エントリポイント
# =====================================================================

def main():
    """
    スクリプトのエントリポイント。

    使用法:
        python circle_flight.py [パラメータファイルパス]

    引数なしの場合、スクリプトと同じディレクトリの
    circle_params.json を読み込む。
    """
    # パラメータファイルパスの決定
    if len(sys.argv) > 1:
        param_path = Path(sys.argv[1])
    else:
        param_path = Path(__file__).parent / "circle_params.json"

    if not param_path.exists():
        print(f"✗ パラメータファイルが見つかりません: {param_path}")
        print("  使用法: python circle_flight.py [params.json]")
        sys.exit(1)

    print(f"パラメータファイル: {param_path}")

    # コントローラー生成（ここでパラメータ読み込みとバリデーション）
    try:
        controller = CircleFlightController(str(param_path))
    except (FileNotFoundError, json.JSONDecodeError, ValueError) as e:
        print(f"✗ パラメータ読み込みエラー: {e}")
        sys.exit(1)

    # SIGINT / SIGTERM ハンドリング（計画ドキュメント 8.3節）
    def handle_interrupt(signum, frame):
        print(f"\n⚠ 割り込み信号({signum})検出 → 安全停止")
        controller._running = False

    signal.signal(signal.SIGINT, handle_interrupt)
    signal.signal(signal.SIGTERM, handle_interrupt)

    # メインシーケンス実行
    controller.run()


if __name__ == "__main__":
    main()
