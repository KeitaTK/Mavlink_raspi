#!/usr/bin/env python3
"""
zigzag_flight.py - Guidedモードでジグザグ（蛇行）飛行を実行するスクリプト

square_flight.py のアーキテクチャを踏襲し、矩形エリア内を対角線セグメントで
折り返しながら横切る真のジグザグパターンを実装する。

JSONパラメータファイルから設定を読み込み、
離陸 → ジグザグ飛行 → 着陸のシーケンスを自動実行する。
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


def generate_edge_waypoints(v_start, v_end, n):
    """
    エッジ上の n 個の waypoint を線形補間で生成する。

    Args:
        v_start: 始点のローカル座標 (x, y)
        v_end: 終点のローカル座標 (x, y)
        n: 分割数（waypoint 数、始点を含む）

    Returns:
        [(x, y), ...] — エッジ上の n 点（t=0 〜 t=1）
    """
    x0, y0 = v_start
    x1, y1 = v_end
    waypoints = []
    for i in range(n):
        t = i / (n - 1) if n > 1 else 0.0
        x = x0 + t * (x1 - x0)
        y = y0 + t * (y1 - y0)
        waypoints.append((x, y))
    return waypoints


def generate_zigzag_vertices(half_width, half_height, num_zigs,
                              zigzag_axis, start_corner):
    """
    対角線ジグザグ飛行の全頂点を訪問順に生成する。

    num_zigs 本の対角線セグメントで矩形を斜めに横切る。
    頂点数 = num_zigs + 1（スタート + 中間折返し点 + 終了）。

    矩形の角の定義（中心を原点とするローカル座標、東=x, 北=y）:
        NE: (+half_width, +half_height)
        NW: (-half_width, +half_height)
        SW: (-half_width, -half_height)
        SE: (+half_width, -half_height)

    Args:
        half_width: 中心から左右の辺までの距離 [m]
        half_height: 中心から上下の辺までの距離 [m]
        num_zigs: 折り返し回数 = セグメント本数（>=2）
        zigzag_axis: "horizontal"（x座標が±half_widthで交互）または
                     "vertical"（y座標が±half_heightで交互）
        start_corner: 開始コーナー "NE", "NW", "SE", "SW"

    Returns:
        [(x, y), ...] — num_zigs+1 個の頂点のローカル座標（訪問順）

    例: num_zigs=3, start_corner="NE", zigzag_axis="horizontal"
        V0: NE (+w, +h)
        V1: (-w, +h/3)   ← 対辺(西)の1/3地点
        V2: (+w, -h/3)   ← 対辺(東)の2/3地点
        V3: SW (-w, -h)  ← 最終頂点
    """
    w = half_width
    h = half_height
    vertices = []

    if zigzag_axis == "horizontal":
        # x座標が±half_widthで交互に振れる
        # y座標が start_corner から対角方向に等間隔で進行
        step_y = 2.0 * h / num_zigs
        north_start = (start_corner in ("NE", "NW"))
        east_start = (start_corner in ("NE", "SE"))

        for i in range(num_zigs + 1):
            # y座標: start側のyから対角方向に進行
            if north_start:
                y = h - i * step_y
            else:
                y = -h + i * step_y

            # x座標: スタート側のxから始め、セグメント毎に対辺のxと交互
            if east_start:
                # 東側スタート: 偶数iは東(+w), 奇数iは西(-w)
                x = w if i % 2 == 0 else -w
            else:
                # 西側スタート: 偶数iは西(-w), 奇数iは東(+w)
                x = -w if i % 2 == 0 else w

            vertices.append((x, y))

    else:  # zigzag_axis == "vertical"
        # y座標が±half_heightで交互に振れる
        # x座標が start_corner から対角方向に等間隔で進行
        step_x = 2.0 * w / num_zigs
        north_start = (start_corner in ("NE", "NW"))
        east_start = (start_corner in ("NE", "SE"))

        for i in range(num_zigs + 1):
            # x座標: start側のxから対角方向に進行
            if east_start:
                x = w - i * step_x
            else:
                x = -w + i * step_x

            # y座標: スタート側のyから始め、セグメント毎に対辺のyと交互
            if north_start:
                # 北側スタート: 偶数iは北(+h), 奇数iは南(-h)
                y = h if i % 2 == 0 else -h
            else:
                # 南側スタート: 偶数iは南(-h), 奇数iは北(+h)
                y = -h if i % 2 == 0 else h

            vertices.append((x, y))

    return vertices


# =====================================================================
#  パラメータ読み込み・バリデーション
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

    # half_width_m のバリデーション
    hw = params.get("half_width_m", 5.0)
    if hw <= 0.0:
        raise ValueError(
            f"half_width_m は正の値である必要があります: {hw}")

    # half_height_m のバリデーション
    hh = params.get("half_height_m", 5.0)
    if hh <= 0.0:
        raise ValueError(
            f"half_height_m は正の値である必要があります: {hh}")

    # speed_m_s のバリデーション
    speed = params.get("speed_m_s", 1.0)
    if speed <= 0.0:
        raise ValueError(
            f"speed_m_s は正の値である必要があります: {speed}")

    # altitude_m のバリデーション
    altitude = params.get("altitude_m", 2.0)
    if altitude < 0.5:
        raise ValueError(
            f"altitude_m が小さすぎます: {altitude}（>= 0.5）")

    # stop_at_turn_sec のバリデーション
    stop_sec = params.get("stop_at_turn_sec", 2.0)
    if stop_sec < 0.0:
        raise ValueError(
            f"stop_at_turn_sec が負の値です: {stop_sec}（>= 0）")

    # loiter_at_start_sec のバリデーション
    loiter_start = params.get("loiter_at_start_sec", 3.0)
    if loiter_start < 0.0:
        raise ValueError(
            f"loiter_at_start_sec が負の値です: {loiter_start}（>= 0）")

    # num_zigs のバリデーション
    num_zigs = params.get("num_zigs", 3)
    if num_zigs < 2:
        raise ValueError(
            f"num_zigs が小さすぎます: {num_zigs}（>= 2）")

    # zigzag_axis のバリデーション
    zigzag_axis = params.get("zigzag_axis", "horizontal")
    if zigzag_axis not in ("horizontal", "vertical"):
        raise ValueError(
            f"zigzag_axis が不正: {zigzag_axis}"
            "（'horizontal' または 'vertical'）")

    # start_corner のバリデーション
    start_corner = params.get("start_corner", "NE")
    if start_corner not in ("NE", "NW", "SE", "SW"):
        raise ValueError(
            f"start_corner が不正: {start_corner}"
            "（'NE' / 'NW' / 'SE' / 'SW'）")

    # yaw_mode のバリデーション
    yaw_mode = params.get("yaw_mode", "fixed")
    if yaw_mode not in ("fixed", "edge", "center"):
        raise ValueError(
            f"yaw_mode が不正: {yaw_mode}"
            "（'fixed' / 'edge' / 'center'）")

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
    params.setdefault("center", {"latitude": 0.0, "longitude": 0.0})
    params.setdefault("half_width_m", 5.0)
    params.setdefault("half_height_m", 5.0)
    params.setdefault("speed_m_s", 1.0)
    params.setdefault("altitude_m", 0.6)
    params.setdefault("stop_at_turn_sec", 2.0)
    params.setdefault("num_zigs", 3)
    params.setdefault("zigzag_axis", "horizontal")
    params.setdefault("start_corner", "NE")
    params.setdefault("yaw_mode", "fixed")
    params.setdefault("fixed_yaw_deg", 0.0)
    params.setdefault("takeoff_alt_m", 0.5)
    params.setdefault("send_rate_hz", 10)
    params.setdefault("mask", "0x09F8")
    params.setdefault("land_after", True)
    params.setdefault("loiter_after_takeoff_sec", 3.0)
    params.setdefault("loiter_before_land_sec", 3.0)
    params.setdefault("loiter_at_start_sec", 3.0)

    return params


# =====================================================================
#  ZigzagFlightController - メインコントローラークラス
# =====================================================================

class ZigzagFlightController:
    """
    Guidedモードでジグザグ飛行を実行するコントローラークラス。

    内部状態遷移:
        WAITING_ARM → TAKEOFF → ZIGZAG_START → ZIGZAG_FLYING
        → ZIGZAG_COMPLETE → LANDING → COMPLETE

    スレッド構成（square_flight.py のパターンを踏襲）:
        - メインスレッド: 飛行シーケンス制御
        - モニタースレッド: HEARTBEAT/GPS監視、ディスアーム検出
        - レコードスレッド: CSVデータ記録
    """

    # ── 状態定数 ──
    STATE_WAITING_ARM = "WAITING_ARM"
    STATE_TAKEOFF = "TAKEOFF"
    STATE_ZIGZAG_START = "ZIGZAG_START"
    STATE_ZIGZAG_FLYING = "ZIGZAG_FLYING"
    STATE_ZIGZAG_COMPLETE = "ZIGZAG_COMPLETE"
    STATE_LANDING = "LANDING"
    STATE_COMPLETE = "COMPLETE"

    # シリアル接続設定（square_flight.py のパターンを踏襲）
    SERIAL_DEVICE = '/dev/ttyAMA0'
    SERIAL_BAUD = 1000000

    # CSV保存ディレクトリ（square_flight.py と同様）
    CSV_DIR = Path.home() / "LOGS_Pixhawk6c"

    # ── モード名マッピング（クラス属性として一度だけ定義）──
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
        JSONパラメータ読み込み → バリデーション → ジグザグ頂点計算。

        Args:
            param_path: JSONパラメータファイルのパス
        """
        # ── パラメータ読み込みと検証 ──
        self.params = load_params(param_path)
        print("✓ パラメータ読み込み完了")
        self._print_params()

        # ── ジグザグ頂点の生成 ──
        self.vertices = generate_zigzag_vertices(
            half_width=self.params["half_width_m"],
            half_height=self.params["half_height_m"],
            num_zigs=self.params["num_zigs"],
            zigzag_axis=self.params["zigzag_axis"],
            start_corner=self.params["start_corner"],
        )
        print(f"✓ ジグザグ頂点生成完了: {len(self.vertices)}点"
              f"（{self.params['num_zigs']}セグメント）")
        for i, (x, y) in enumerate(self.vertices):
            label = "開始" if i == 0 else ("終了" if i == len(self.vertices) - 1 else "折返し")
            print(f"    V{i}: ({x:+.3f}, {y:+.3f})m"
                  f"  [{label}]")

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
        self._data_records = []          # CSV記録用データ
        self._state = self.STATE_WAITING_ARM
        self._origin = None              # 初回GPS位置
        self._takeoff_lat = None         # 離陸地点の緯度 (degree)
        self._takeoff_lon = None         # 離陸地点の経度 (degree)

        # ── デバッグ用カウンタ ──
        self._monitor_loop_count = 0
        self._monitor_hb_count = 0
        self._monitor_pos_count = 0

        # ── 基準点: 矩形の中心座標 ──
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
        print(f"  矩形サイズ: 幅={p['half_width_m']*2}m x 高さ={p['half_height_m']*2}m")
        print(f"  高度: {p['altitude_m']}m | 速度: {p['speed_m_s']}m/s")
        print(f"  ジグザグ数: {p['num_zigs']} | 軸: {p['zigzag_axis']}"
              f" | 開始コーナー: {p['start_corner']}")
        print(f"  折返し停止: {p['stop_at_turn_sec']}秒 | 送信レート: {p['send_rate_hz']}Hz")
        print(f"  開始位置待機: {p['loiter_at_start_sec']}秒")
        print(f"  ヨーモード: {p['yaw_mode']} | land_after: {p['land_after']}")

    # ──────────────────────────────────────────
    #  MAVLink 接続（square_flight.py のパターンを踏襲）
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

        # MAVLink接続（square_flight.py のパターンを踏襲）
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
        print(f"[INFO] target_system={self.master.target_system},"
              f" target_component={self.master.target_component}")
        print(f"[INFO] メッセージレート設定:"
              f" GPS_RAW_INT(24), GLOBAL_POSITION_INT(33), ATTITUDE(30) → 5Hz")

        # スレッド起動（square_flight.py のパターンを踏襲）
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

        # 高度デバッグスレッド起動
        self._altitude_debug_thread = threading.Thread(
            target=self._altitude_debug_loop, daemon=True,
            name='alt_debug'
        )
        self._altitude_debug_thread.start()
        print("✓ 高度デバッグスレッド起動")

        return True

    # ──────────────────────────────────────────
    #  モニタースレッド（square_flight.py のパターンを踏襲）
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
                msg.base_mode
                & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
            )

            with self._io_lock:
                prev_armed = self._is_armed
                self._is_armed = is_armed
                self._is_guided = is_guided
                self._is_land_mode = is_land_mode
                current_state = self._state

            # HEARTBEAT デバッグ: 約1秒毎にモード情報表示
            if self._monitor_hb_count % 10 == 0:
                mode_str = self.MODE_NAMES.get(
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
                return

            # モード変更検出（飛行中にGuided/LAND以外になった場合）
            if current_state not in (
                self.STATE_WAITING_ARM,
                self.STATE_COMPLETE,
            ):
                # LANDING状態ではLANDモード(9)を許可
                if current_state == self.STATE_LANDING:
                    if not is_land_mode and not is_guided:
                        print(
                            "\n⚠ モード変更検出"
                            "（着陸中にLAND/Guided以外）→ 安全停止"
                        )
                        self._running = False
                        return
                elif not is_guided:
                    print(
                        "\n⚠ モード変更検出"
                        "（Guided→他モード）→ 安全停止"
                    )
                    self._running = False
                    return

        elif msg_type == "GLOBAL_POSITION_INT":
            self._monitor_pos_count += 1
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.relative_alt / 1000.0  # 相対高度 [m]

            # 原点設定（初回のみ）
            if self._origin is None:
                self._origin = msg
                self._takeoff_lat = lat   # 離陸地点の緯度
                self._takeoff_lon = lon   # 離陸地点の経度
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
                self._gps_now.update({
                    'x': x, 'y': y, 'z': z,
                    'lat': lat, 'lon': lon, 'alt': alt,
                })

    def _monitor_loop(self):
        """
        HEARTBEAT/GPS監視ループ（バッファドレイン方式）。

        square_flight.py のモニター方式を踏襲:
           1. バッファ内の全メッセージを一気に処理（blocking=False）
           2. 新着メッセージを短時間待機（最大20ms）
           3. 続けて残りのバッファも処理
           4. send_hz に基づいてスリープ
        """
        send_hz = self.params["send_rate_hz"]

        while self._running:
            self._monitor_loop_count += 1

            # ── ステップ1: バッファ内の全メッセージを一気に処理 ──
            while True:
                msg = self.master.recv_match(blocking=False)
                if msg is None:
                    break
                self._process_message(msg)

            # ── ステップ2: 新着メッセージを短時間待機（最大20ms）──
            msg = self.master.recv_match(blocking=True, timeout=0.02)
            if msg is not None:
                self._process_message(msg)

                # ── ステップ3: 続けて残りのバッファも処理 ──
                while True:
                    msg = self.master.recv_match(blocking=False)
                    if msg is None:
                        break
                    self._process_message(msg)

            # ── デバッグ: 約5秒毎にモニターサマリ出力 ──
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
    #  レコードスレッド（square_flight.py のパターンを踏襲）
    # ──────────────────────────────────────────

    def _record_loop(self):
        """
        CSVデータ記録ループ。
        square_flight.py と同一フォーマットで、send_rate_hz の間隔で記録する。

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
    #  高度デバッグスレッド
    # ──────────────────────────────────────────

    def _altitude_debug_loop(self):
        """
        高度デバッグループ（2Hz）。

        _gps_now['z'] と _target['z'] を _io_lock 下で読み取り、
        0.5秒間隔で差分を表示する。
        _running == False で停止。
        """
        while self._running:
            with self._io_lock:
                actual_z = self._gps_now['z']
                target_z = self._target['z']
            diff = actual_z - target_z
            sign = '+' if diff >= 0 else ''
            print(
                f"[ALT_DEBUG] actual={actual_z:.2f}m"
                f" target={target_z:.2f}m"
                f" diff={sign}{diff:.2f}m"
            )
            time.sleep(0.5)

    # ──────────────────────────────────────────
    #  セットポイント送信（square_flight.py のパターンを踏襲）
    # ──────────────────────────────────────────

    def _send_setpoint(self, lat_deg, lon_deg, alt_m, yaw_deg):
        """
        位置目標を MAVLink で送信する。

        set_position_target_global_int_send を使用し、
        マスクで制御対象を指定する。

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
    #  ヨー角計算（square_flight.py のパターンを踏襲）
    # ──────────────────────────────────────────

    def _calc_yaw(self, v_start, v_end):
        """
        ヨー角を yaw_mode に応じて計算する。

        Args:
            v_start: 始点のローカル座標 (x, y)
            v_end: 終点のローカル座標 (x, y)

        Returns:
            yaw_deg: ヨー角 [deg]（北=0°, 東=90°）
        """
        yaw_mode = self.params["yaw_mode"]

        if yaw_mode == "fixed":
            return self.params["fixed_yaw_deg"]

        elif yaw_mode == "edge":
            # エッジの進行方向を向く
            dx = v_end[0] - v_start[0]  # 東方向
            dy = v_end[1] - v_start[1]  # 北方向
            # atan2(dx, dy): 北=0°, 東=90°
            return math.degrees(math.atan2(dx, dy)) % 360

        else:  # center
            # 矩形の中心を向く
            with self._io_lock:
                x_now = self._gps_now['x']
                y_now = self._gps_now['y']
            # 現在位置 → 中心 (0, 0) への方位
            dx = -x_now   # 中心への東方向ベクトル
            dy = -y_now   # 中心への北方向ベクトル
            return math.degrees(math.atan2(dx, dy)) % 360

    # ──────────────────────────────────────────
    #  離陸 / 着陸指令（square_flight.py のパターンを踏襲）
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
        """
        LANDモードに切り替えて着陸する。

        mavutil の set_mode() を使用（MAV_CMD_DO_SET_MODE より確実）。
        確認のため 0.5 秒後に再送する。
        """
        self.master.set_mode(9)  # LAND mode (custom_mode = 9)
        time.sleep(0.5)
        self.master.set_mode(9)  # 確認のため再送

    def _send_land_fallback(self):
        """
        GUIDEDモードのまま高度0に降下させる（LANDモード切替失敗時のフォールバック）。

        現在のGPS位置を維持しつつ、高度目標を0mに設定する。
        """
        with self._io_lock:
            lat = self._gps_now.get('lat', 0.0)
            lon = self._gps_now.get('lon', 0.0)

        if lat == 0.0 and lon == 0.0:
            # GPS未受信の場合は中心座標に降下
            lat = self._ref_lat
            lon = self._ref_lon
            print("⚠ GPS未受信のため中心座標に降下します")

        print(f"  [FALLBACK] GUIDED降下: lat={lat:.7f}, lon={lon:.7f}, alt=0.0m")
        self._send_setpoint(lat, lon, 0.0, 0.0)

    # ──────────────────────────────────────────
    #  飛行シーケンス（square_flight.py のパターンを踏襲）
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

        # 上昇トレンド検出用変数
        prev_z = 0.0
        rising_count = 0

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

            # 離陸判定: 高度が閾値に達した
            if current_z >= takeoff_alt * 0.9:
                print(f"✓ 離陸高度到達: {current_z:.2f}m")
                return True

            # 上昇トレンド検出: 離陸指令から5秒経過後
            if elapsed > 5.0:
                if current_z > prev_z + 0.05:  # 0.05m以上の上昇
                    rising_count += 1
                    if rising_count >= 3:  # 3回連続上昇 → 離陸成功とみなす
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

            # 3秒間隔でTAKEOFFコマンドを再送（初回が無視された場合のリカバリ）
            if time.time() - last_retry >= 3.0:
                self._send_takeoff(takeoff_alt)
                last_retry = time.time()

            time.sleep(0.1)

        return False

    def move_to_start_vertex(self):
        """
        最初の頂点（ジグザグの開始位置）へ移動する。

        - 目標高度 altitude_m に到達するまで待機
        - 開始頂点から閾値以内に近づくまで待機
        - 到達前に定期的にセットポイントを再送信（3秒間隔）
        - 到達後、loiter_at_start_sec 秒間ホバリング待機

        Returns:
            bool: 到達成功時 True
        """
        start_corner = self.params["start_corner"]
        print(
            f"\n--- [{self.STATE_ZIGZAG_START}]"
            f" ジグザグ開始位置（{start_corner}）へ移動 ---"
        )
        self._state = self.STATE_ZIGZAG_START

        # 最初の頂点（V0）のローカル座標をGPSに変換
        v0_x, v0_y = self.vertices[0]
        lat_v0, lon_v0, _ = local_xyz_to_gps(
            v0_x, v0_y, self.params["altitude_m"],
            self._ref_lat, self._ref_lon, self._ref_alt,
        )

        yaw = self._calc_yaw((0, 0), (v0_x, v0_y))
        self._send_setpoint(lat_v0, lon_v0, self.params["altitude_m"], yaw)
        print(
            f"  目標頂点V0: lat={lat_v0:.7f}, lon={lon_v0:.7f},"
            f" alt={self.params['altitude_m']}m, yaw={yaw:.1f}deg"
        )

        # 到達閾値: 矩形の対角線の20%
        hw = self.params["half_width_m"]
        hh = self.params["half_height_m"]
        diagonal = math.sqrt((2 * hw) ** 2 + (2 * hh) ** 2)
        threshold = max(diagonal * 0.2, 0.3)
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
            if not alt_reached and z >= target_alt * 0.95:
                alt_reached = True
                print(f"✓ 目標高度到達: {z:.2f}m")

            # 水平位置チェック（高度到達後に評価）
            if alt_reached:
                dist = math.sqrt((x - v0_x) ** 2 + (y - v0_y) ** 2)
                if dist < threshold:
                    print(
                        f"✓ 開始位置（V0）到達: 距離={dist:.2f}m"
                        f"（閾値={threshold:.2f}m）"
                    )
                    # 開始位置到着後のホバリング待機
                    loiter_sec = self.params.get("loiter_at_start_sec", 3.0)
                    if loiter_sec > 0.0:
                        send_hz = self.params["send_rate_hz"]
                        dt = 1.0 / send_hz
                        print(
                            f"  開始位置で {loiter_sec:.1f}秒"
                            f" ホバリング待機..."
                        )
                        loiter_start = time.time()
                        while time.time() - loiter_start < loiter_sec:
                            if not self._running:
                                return False
                            self._send_setpoint(
                                lat_v0, lon_v0,
                                self.params["altitude_m"], yaw
                            )
                            time.sleep(dt)
                        print("  ✓ 開始位置待機完了")
                    return True

            # 定期的に目標を再送信（3秒間隔）
            now = time.time()
            if now - last_send >= 3.0:
                self._send_setpoint(lat_v0, lon_v0,
                                    self.params["altitude_m"], yaw)
                last_send = now

            # タイムアウト
            if now - start > timeout:
                dist = math.sqrt((x - v0_x) ** 2 + (y - v0_y) ** 2)
                print(
                    f"⚠ タイムアウト: 開始位置到達"
                    f"（{timeout}秒, 距離={dist:.2f}m）— 続行します"
                )
                return True  # タイムアウトでも飛行続行

            time.sleep(0.5)

        return False

    # ──────────────────────────────────────────
    #  ジグザグ飛行 コアメソッド
    # ──────────────────────────────────────────

    def execute_edge(self, v_start, v_end):
        """
        1エッジの飛行を実行する（方式A: 位置補間）。

        エッジ V_start → V_end を線形補間で細かく分割し、
        send_rate_hz のレートで連続送信する。

        Args:
            v_start: 始点のローカル座標 (x, y)
            v_end: 終点のローカル座標 (x, y)

        Returns:
            bool: 成功時 True
        """
        dx = v_end[0] - v_start[0]
        dy = v_end[1] - v_start[1]
        edge_length = math.sqrt(dx**2 + dy**2)
        edge_time = edge_length / self.params["speed_m_s"]

        send_hz = self.params["send_rate_hz"]
        n_points = max(2, int(math.ceil(edge_time * send_hz)))
        dt = 1.0 / send_hz

        altitude = self.params["altitude_m"]

        print(
            f"    エッジ: ({v_start[0]:+.3f},{v_start[1]:+.3f}) →"
            f" ({v_end[0]:+.3f},{v_end[1]:+.3f})"
        )
        print(
            f"    長さ={edge_length:.3f}m, 時間={edge_time:.2f}秒,"
            f" 分割数={n_points}, dt={dt:.3f}秒"
        )

        # エッジ上の waypoint を生成（線形補間）
        wps = generate_edge_waypoints(v_start, v_end, n_points)

        edge_start = time.time()
        wp_send_count = 0

        for i, (x, y) in enumerate(wps):
            if not self._running:
                return False

            # ローカル座標 → GPS座標
            lat, lon, _ = local_xyz_to_gps(
                x, y, altitude,
                self._ref_lat, self._ref_lon, self._ref_alt
            )

            # ヨー角計算
            yaw = self._calc_yaw(v_start, v_end)

            # セットポイント送信
            self._send_setpoint(lat, lon, altitude, yaw)
            wp_send_count += 1

            # 5回に1回、現在高度と目標高度を表示
            if wp_send_count % 5 == 0:
                with self._io_lock:
                    cz = self._gps_now['z']
                print(f"      [ALT] z={cz:.2f}m target={altitude:.2f}m")

            # 次の送信タイミングまで待機
            next_time = edge_start + (i + 1) * dt
            sleep_time = next_time - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)

        elapsed = time.time() - edge_start
        print(f"    ✓ エッジ完了（{elapsed:.2f}秒, {wp_send_count}回送信）")
        return True

    def stop_at_turn(self, vertex, duration, label="折返し停止"):
        """
        指定頂点で指定時間停止する。

        頂点の座標を send_rate_hz で繰り返し送信し、
        ArduPilot の位置ホバリング制御に任せる。

        Args:
            vertex: 頂点のローカル座標 (x, y)
            duration: 停止時間 [秒]
            label: 表示ラベル（デフォルト: "折返し停止"）

        Returns:
            bool: 成功時 True
        """
        if duration <= 0:
            return True

        x, y = vertex
        altitude = self.params["altitude_m"]
        lat, lon, _ = local_xyz_to_gps(
            x, y, altitude,
            self._ref_lat, self._ref_lon, self._ref_alt
        )

        send_hz = self.params["send_rate_hz"]
        dt = 1.0 / send_hz
        yaw = self.params["fixed_yaw_deg"]

        print(f"    {label} {duration:.1f}秒...")

        stop_start = time.time()
        while time.time() - stop_start < duration:
            if not self._running:
                return False
            self._send_setpoint(lat, lon, altitude, yaw)
            time.sleep(dt)

        print(f"    ✓ {label}完了")
        return True

    def execute_zigzag_flight(self):
        """
        ジグザグ飛行のメインループを実行する。

        全頂点を順に訪問するエッジ飛行を実行する。
        頂点数 = num_zigs + 1、エッジ数 = num_zigs
        （各エッジは矩形の対角線セグメント）

        中間折返し頂点では折返し停止（stop_at_turn_sec）、
        最終頂点では待機停止（loiter_at_start_sec）。

        Returns:
            bool: 飛行成功時 True
        """
        print(
            f"\n--- [{self.STATE_ZIGZAG_FLYING}] ジグザグ飛行実行 ---"
        )
        self._state = self.STATE_ZIGZAG_FLYING

        num_zigs = self.params["num_zigs"]
        stop_time = self.params["stop_at_turn_sec"]
        loiter_time = self.params["loiter_at_start_sec"]
        zigzag_axis = self.params["zigzag_axis"]

        # 総移動距離と推定時間の計算
        total_edge_length = 0.0
        num_edges = len(self.vertices) - 1  # = num_zigs
        for i in range(num_edges):
            v_start = self.vertices[i]
            v_end = self.vertices[i + 1]
            dx = v_end[0] - v_start[0]
            dy = v_end[1] - v_start[1]
            total_edge_length += math.sqrt(dx**2 + dy**2)

        # 停止回数: 中間折返し (num_edges-1) + 最終頂点 (1)
        num_turn_stops = num_edges - 1  # 中間折返し
        estimated_total_time = (
            total_edge_length / self.params["speed_m_s"]
            + stop_time * num_turn_stops
            + loiter_time  # 最終頂点待機
        )
        print(f"  矩形サイズ: {self.params['half_width_m']*2}m"
              f" x {self.params['half_height_m']*2}m")
        print(f"  ジグザグ数: {num_zigs} | 軸: {zigzag_axis}")
        print(f"  開始コーナー: {self.params['start_corner']}")
        print(f"  速度: {self.params['speed_m_s']}m/s"
              f" | 折返し停止: {stop_time}秒"
              f" | 最終待機: {loiter_time}秒")
        print(f"  総エッジ数: {num_edges}")
        print(f"  推定総移動距離: {total_edge_length:.1f}m")
        print(f"  推定総時間: {estimated_total_time:.1f}秒")

        flight_start = time.time()

        for i in range(num_edges):
            if not self._running:
                print("⚠ 飛行中断")
                return False

            v_start = self.vertices[i]
            v_end = self.vertices[i + 1]

            # 各エッジは対角線セグメント（全エッジ統一）
            seg_id = i + 1
            edge_type = f"セグメント{seg_id}"

            print(f"\n  エッジ {i}: {edge_type}"
                  f" V{i} → V{i + 1}")
            if not self.execute_edge(v_start, v_end):
                return False

            # 最終エッジ: loiter_at_start_sec で待機、
            # 中間折返し: stop_at_turn_sec で折返し停止
            is_last = (i == num_edges - 1)
            wait_sec = loiter_time if is_last else stop_time
            label = "最終頂点待機" if is_last else "折返し停止"

            if wait_sec > 0:
                if not self.stop_at_turn(v_end, wait_sec, label=label):
                    return False

            if is_last:
                print(f"  ✓ 最終頂点 V{i + 1} 到達"
                      f"（{edge_type}終点、{loiter_time}秒待機完了）")
            else:
                print(f"  ✓ 頂点 V{i + 1} 到達（{edge_type}終点）")

        total_elapsed = time.time() - flight_start
        print(f"\n✓ ジグザグ飛行完了（{total_elapsed:.1f}秒）")
        self._state = self.STATE_ZIGZAG_COMPLETE
        return True

    # ──────────────────────────────────────────
    #  帰還 / 着陸（square_flight.py のパターンを踏襲）
    # ──────────────────────────────────────────

    def return_to_takeoff(self):
        """
        離陸地点の上空（現在の飛行高度を維持）へ戻る。

        離陸地点の水平座標 + 現在の飛行高度を目標として送信し、
        十分近づくまで待機する。
        """
        print(f"\n--- [{self.STATE_ZIGZAG_COMPLETE}] 離陸地点上空へ帰還 ---")

        altitude = self.params["altitude_m"]  # 現在の飛行高度を維持

        print(f"  目標: lat={self._takeoff_lat:.7f},"
              f" lon={self._takeoff_lon:.7f}, alt={altitude}m")

        # セットポイント送信
        self._send_setpoint(self._takeoff_lat, self._takeoff_lon, altitude,
                            self.params["fixed_yaw_deg"])

        # 離陸地点に近づくまで待機
        hw = self.params["half_width_m"]
        hh = self.params["half_height_m"]
        diagonal = math.sqrt((2 * hw) ** 2 + (2 * hh) ** 2)
        threshold = max(diagonal * 0.2, 0.3)  # 最低30cm
        timeout = 30.0
        start = time.time()

        while self._running:
            with self._io_lock:
                gps_x = self._gps_now['x']
                gps_y = self._gps_now['y']

            # 離陸地点のローカル座標を基準に計算
            takeoff_x, takeoff_y, _ = gps_to_local_xyz(
                self._takeoff_lat, self._takeoff_lon, 0,
                self._ref_lat, self._ref_lon, self._ref_alt
            )
            dist = math.sqrt((gps_x - takeoff_x)**2 + (gps_y - takeoff_y)**2)

            if dist < threshold:
                print(f"✓ 離陸地点上空に到達: 距離={dist:.2f}m")
                return True

            if time.time() - start > timeout:
                print(f"⚠ タイムアウト: 離陸地点帰還"
                      f"（{timeout}秒, 距離={dist:.2f}m）")
                return False

            # 3秒毎にセットポイント再送
            if int(time.time() - start) % 3 == 0:
                self._send_setpoint(self._takeoff_lat, self._takeoff_lon,
                                    altitude, self.params["fixed_yaw_deg"])

            time.sleep(0.5)

        return False

    def land(self):
        """
        着陸シーケンスを実行する。

        1. land_after=False の場合は着陸をスキップ
        2. loiter_before_land_sec 秒の安定待機
        3. set_mode(LAND) で LANDモードに切り替え
        4. 以下の複合条件で着陸完了を判定:
           a. ディスアーム検出（_is_armed == False）
           b. 高度が 0.1m 未満で5秒間安定
           c. LANDモード検出から60秒経過（タイムアウト）
        5. LANDモード切替が10秒以内に検出されない場合:
           GUIDEDモードのまま高度0に降下（フォールバック）

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

        # 着陸指令送信（set_mode で LANDモードに切り替え）
        self._send_land()
        print("✓ 着陸指令送信（set_mode LAND）")

        # 着陸検出待機（複合条件 + フォールバック）
        FALLBACK_TIMEOUT = 10.0          # LANDモード未検出時のフォールバック猶予
        OVERALL_TIMEOUT = 60.0           # 全体タイムアウト
        start = time.time()
        land_mode_detected_time = None   # LANDモード検出時刻
        ground_stable_start = None       # 高度安定検出開始時刻
        prev_z = None
        fallback_active = False          # フォールバック発動中
        last_fallback_send = 0.0         # 直近のフォールバックセットポイント送信時刻

        while self._running:
            with self._io_lock:
                armed = self._is_armed
                is_land = self._is_land_mode
                current_z = self._gps_now['z']

            # 条件1: ディスアーム検出
            if not armed:
                print("✓ 着陸+ディスアーム完了")
                self._state = self.STATE_COMPLETE
                return True

            # LANDモード検出時刻を記録
            if is_land and land_mode_detected_time is None:
                land_mode_detected_time = time.time()
                print(f"  LANDモード検出（z={current_z:.2f}m）")

            # ── フォールバック判定 ──
            # LANDモードが10秒以内に検出されなければGUIDED降下に切り替え
            elapsed = time.time() - start
            if (not fallback_active
                    and land_mode_detected_time is None
                    and elapsed >= FALLBACK_TIMEOUT):
                print(
                    f"\n⚠ LANDモード未検出（{FALLBACK_TIMEOUT}秒経過）"
                    " → GUIDED降下フォールバック"
                )
                fallback_active = True

            # フォールバック発動中: 2秒間隔で高度0のセットポイントを送信
            if fallback_active:
                now = time.time()
                if now - last_fallback_send >= 2.0:
                    self._send_land_fallback()
                    last_fallback_send = now
                    # GUIDED降下中に高度が0.2m未満になったら着陸完了とみなす
                    if current_z < 0.2:
                        if ground_stable_start is None:
                            ground_stable_start = now
                        elif now - ground_stable_start >= 3.0:
                            print(
                                f"✓ GUIDED降下完了: z={current_z:.2f}m"
                                f"（3秒間低高度安定）"
                            )
                            self._state = self.STATE_COMPLETE
                            return True
                    else:
                        ground_stable_start = None
                time.sleep(0.5)
                continue

            # ── LANDモード正常時の着陸判定 ──

            # 条件2: 高度が0.1m未満で5秒間安定
            if current_z < 0.1:
                if prev_z is None:
                    prev_z = current_z
                    ground_stable_start = time.time()
                elif abs(current_z - prev_z) < 0.02:
                    # 高度変化がほぼなし
                    if ground_stable_start is not None:
                        if time.time() - ground_stable_start >= 5.0:
                            print(
                                f"✓ 着陸高度安定検出: z={current_z:.2f}m"
                                f"（5秒間安定）→ 着陸完了"
                            )
                            self._state = self.STATE_COMPLETE
                            return True
                else:
                    # 高度が動いた → リセット
                    ground_stable_start = time.time()
                prev_z = current_z
            else:
                # 高度が0.1m以上 → リセット
                ground_stable_start = None
                prev_z = current_z

            # 条件3: LANDモード検出から60秒経過（タイムアウト）
            if land_mode_detected_time is not None:
                if time.time() - land_mode_detected_time >= 60.0:
                    print(
                        "✓ LANDモード60秒経過 → 着陸完了とみなす"
                    )
                    self._state = self.STATE_COMPLETE
                    return True

            # 全体タイムアウト（60秒）
            if time.time() - start > OVERALL_TIMEOUT:
                print(
                    f"⚠ タイムアウト: 着陸完了待機（{OVERALL_TIMEOUT}秒）"
                )
                self._state = self.STATE_COMPLETE
                return True

            time.sleep(0.5)

        return False

    # ──────────────────────────────────────────
    #  メイン実行 / クリーンアップ（square_flight.py のパターンを踏襲）
    # ──────────────────────────────────────────

    def run(self):
        """
        メイン飛行シーケンスを実行する。

        シーケンス:
            1. MAVLink接続確立
            2. Guidedモード + Arm 検出
            3. 離陸
            4. 最初の頂点 V0 へ移動
            5. ジグザグ飛行実行
            6. 離陸地点帰還 → 2秒待機 → 着陸
        """
        print("=" * 60)
        print("  ArduPilot ジグザグ飛行制御 - Zigzag Flight")
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

            # 4. ジグザグ開始位置（V0）へ移動
            if not self.move_to_start_vertex():
                print("\n✗ 開始位置移動失敗 → 着陸試行")
                self.land()
                return

            # 5. ジグザグ飛行実行
            if not self.execute_zigzag_flight():
                print("\n✗ ジグザグ飛行中断 → 着陸試行")
                self.land()
                return

            # 6. 離陸地点上空へ帰還 → 着陸
            if self.params.get("land_after", True):
                self.return_to_takeoff()   # 離陸地点上空へ移動
                # 2秒間 takeoff位置のsetpointを0.5秒間隔で送信（GUID_TIMEOUT防止）
                _start = time.time()
                while time.time() - _start < 2.0:
                    self._send_setpoint(self._takeoff_lat, self._takeoff_lon,
                                        self.params["altitude_m"],
                                        self.params["fixed_yaw_deg"])
                    time.sleep(0.5)
                self.land()                 # 着陸
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
            (self._altitude_debug_thread, "altitude_debug"),
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
    #  CSV保存（square_flight.py のパターンを踏襲）
    # ──────────────────────────────────────────

    def _save_csv(self):
        """
        記録データをCSVファイルに保存する。

        保存先: ~/LOGS_Pixhawk6c/{YYYYMMDD_HHMMSS}_zigzag.csv
        フォーマット: Time, GPS_X, GPS_Y, GPS_Z, Target_X, Target_Y, Target_Z
        """
        if not self._data_records:
            print("⚠ 記録データなし")
            return

        now = datetime.datetime.now(
            pytz.timezone("Asia/Tokyo")
        ).strftime("%Y%m%d_%H%M%S")
        path = self.CSV_DIR / f"{now}_zigzag.csv"

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
        python zigzag_flight.py [パラメータファイルパス]

    引数なしの場合、スクリプトと同じディレクトリの
    zigzag_params.json を読み込む。
    """
    # パラメータファイルパスの決定
    if len(sys.argv) > 1:
        param_path = Path(sys.argv[1])
    else:
        param_path = Path(__file__).parent / "zigzag_params.json"

    if not param_path.exists():
        print(f"✗ パラメータファイルが見つかりません: {param_path}")
        print("  使用法: python zigzag_flight.py [params.json]")
        sys.exit(1)

    print(f"パラメータファイル: {param_path}")

    # コントローラー生成（ここでパラメータ読み込みとバリデーション）
    try:
        controller = ZigzagFlightController(str(param_path))
    except (FileNotFoundError, json.JSONDecodeError, ValueError) as e:
        print(f"✗ パラメータ読み込みエラー: {e}")
        sys.exit(1)

    # SIGINT / SIGTERM ハンドリング
    def handle_interrupt(signum, frame):
        print(f"\n⚠ 割り込み信号({signum})検出 → 安全停止")
        controller._running = False

    signal.signal(signal.SIGINT, handle_interrupt)
    signal.signal(signal.SIGTERM, handle_interrupt)

    # メインシーケンス実行
    controller.run()


if __name__ == "__main__":
    main()
