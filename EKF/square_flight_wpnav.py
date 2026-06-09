#!/usr/bin/env python3
"""
square_flight_wpnav.py - WPNAV互換モードで四角形（矩形）飛行を実行するスクリプト

square_flight.py からの派生。
core 変更点: execute_square_flight() を WPNAV（MAV_CMD_NAV_WAYPOINT 単発送信 +
到着確認 → 次頂点）方式に置き換え。

旧: execute_edge() でエッジを細かく分割し10Hz連続送信 + stop_at_vertex() で頂点ホバリング
新: 4頂点だけを MAV_CMD_NAV_WAYPOINT で1回ずつ送信 → 到着確認 → 次頂点
    - WPNAVが頂点間の移動を自律で処理（加速・巡航・減速）
    - 頂点到着後、stop_at_vertex_sec 秒だけ待機してから次頂点へ

到着判定:
  - 距離 < WPNAV_RADIUS (10cm) で到着
  - 距離が増加に転じたら通過とみなす (fly-by)
  - タイムアウト付き

継承（変更なし）:
  - 全ヘルパー関数、connect、モニター/レコードスレッド、離陸/着陸、cleanup、main
  - _send_setpoint, _calc_yaw, stop_at_vertex, move_to_start_vertex

パラメータファイルは square_params.json を共用。
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


def generate_square_vertices(half_width, half_height, direction):
    """
    矩形の4頂点を訪問順に生成する。

    頂点定義（中心を原点とするローカル座標、東=x, 北=y）:
        V0: (+half_width, +half_height)  ← 北東（開始点）
        V1: (+half_width, -half_height)  ← 南東
        V2: (-half_width, -half_height)  ← 南西
        V3: (-half_width, +half_height)  ← 北西

    Args:
        half_width: 中心から左右の辺までの距離 [m]
        half_height: 中心から上下の辺までの距離 [m]
        direction: "CW"（時計回り: V0→V3→V2→V1） または
                   "CCW"（反時計回り: V0→V1→V2→V3）

    Returns:
        [(x, y), ...] — 4頂点のローカル座標（訪問順）
    """
    V0 = (+half_width, +half_height)   # 北東
    V1 = (+half_width, -half_height)   # 南東
    V2 = (-half_width, -half_height)   # 南西
    V3 = (-half_width, +half_height)   # 北西

    if direction == "CCW":
        return [V0, V1, V2, V3]   # 反時計回り
    else:  # CW
        return [V0, V3, V2, V1]   # 時計回り


def generate_edge_waypoints(v_start, v_end, n):
    """
    エッジ上の n 個の waypoint を線形補間で生成する。

    （WPNAVモードでは使用しないが、square_flight.py からの継承として保持）

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


# =====================================================================
#  パラメータ読み込み・バリデーション
# =====================================================================

def _validate_params(params):
    """
    パラメータのバリデーションを行う。
    すべてのパラメータは必須。デフォルト値は使用しない。

    Raises:
        ValueError: パラメータが制約を満たさない場合
    """
    # center のバリデーション
    center = params["center"]
    lat = center["latitude"]
    lon = center["longitude"]
    if not (-90.0 <= lat <= 90.0):
        raise ValueError(
            f"center.latitude が範囲外: {lat}（-90.0 〜 90.0）")
    if not (-180.0 <= lon <= 180.0):
        raise ValueError(
            f"center.longitude が範囲外: {lon}（-180.0 〜 180.0）")

    # half_width_m のバリデーション
    hw = params["half_width_m"]
    if hw <= 0.0:
        raise ValueError(
            f"half_width_m は正の値である必要があります: {hw}")

    # half_height_m のバリデーション
    hh = params["half_height_m"]
    if hh <= 0.0:
        raise ValueError(
            f"half_height_m は正の値である必要があります: {hh}")

    # speed_m_s のバリデーション
    speed = params["speed_m_s"]
    if speed <= 0.0:
        raise ValueError(
            f"speed_m_s は正の値である必要があります: {speed}")

    # altitude_m のバリデーション
    altitude = params["altitude_m"]
    if altitude < 0.5:
        raise ValueError(
            f"altitude_m が小さすぎます: {altitude}（>= 0.5）")

    # stop_at_vertex_sec のバリデーション
    stop_sec = params["stop_at_vertex_sec"]
    if stop_sec < 0.0:
        raise ValueError(
            f"stop_at_vertex_sec が負の値です: {stop_sec}（>= 0）")

    # num_laps のバリデーション
    num_laps = params["num_laps"]
    if num_laps < 1:
        raise ValueError(
            f"num_laps が小さすぎます: {num_laps}（>= 1）")

    # direction のバリデーション
    direction = params["direction"]
    if direction not in ("CW", "CCW"):
        raise ValueError(
            f"direction が不正: {direction}（'CW' または 'CCW'）")

    # yaw_mode のバリデーション
    yaw_mode = params["yaw_mode"]
    if yaw_mode not in ("fixed", "edge", "center"):
        raise ValueError(
            f"yaw_mode が不正: {yaw_mode}"
            "（'fixed' / 'edge' / 'center'）")

    # fixed_yaw_deg のバリデーション
    fixed_yaw = params["fixed_yaw_deg"]
    if not (0.0 <= fixed_yaw <= 360.0):
        raise ValueError(
            f"fixed_yaw_deg が範囲外: {fixed_yaw}（0.0 〜 360.0）")

    # send_rate_hz の最低限チェック
    send_hz = params["send_rate_hz"]
    if send_hz < 1:
        raise ValueError(
            f"send_rate_hz が小さすぎます: {send_hz}（>= 1）")

    # takeoff_alt_m のバリデーション
    takeoff_alt = params["takeoff_alt_m"]
    if takeoff_alt < 0.3:
        raise ValueError(
            f"takeoff_alt_m が小さすぎます: {takeoff_alt}（>= 0.3）")

    # wp_timeout のバリデーション
    wp_timeout = params["wp_timeout"]
    if wp_timeout <= 0.0:
        raise ValueError(
            f"wp_timeout は正の値である必要があります: {wp_timeout}")

    # wpnav_flyby_count のバリデーション
    flyby = params["wpnav_flyby_count"]
    if flyby < 1:
        raise ValueError(
            f"wpnav_flyby_count が小さすぎます: {flyby}（>= 1）")

    # loiter_after_takeoff_sec のバリデーション
    loiter_takeoff = params["loiter_after_takeoff_sec"]
    if loiter_takeoff < 0.0:
        raise ValueError(
            f"loiter_after_takeoff_sec が負の値です: {loiter_takeoff}（>= 0）")

    # loiter_before_land_sec のバリデーション
    loiter_land = params["loiter_before_land_sec"]
    if loiter_land < 0.0:
        raise ValueError(
            f"loiter_before_land_sec が負の値です: {loiter_land}（>= 0）")


def load_params(path):
    """
    JSONパラメータファイルを読み込み、バリデーションを行う。
    すべてのキーが必須。デフォルト値やフォールバックは使用しない。

    Args:
        path: JSONファイルのパス

    Returns:
        バリデーション済みのパラメータ辞書

    Raises:
        FileNotFoundError: ファイルが存在しない場合
        ValueError: 必須キー不足、またはパラメータが制約を満たさない場合
        json.JSONDecodeError: JSONフォーマット不正
    """
    with open(path, 'r') as f:
        params = json.load(f)

    # 必須キーのチェック（全キー必須）
    MANDATORY_KEYS = [
        "center",
        "half_width_m",
        "half_height_m",
        "speed_m_s",
        "altitude_m",
        "stop_at_vertex_sec",
        "num_laps",
        "direction",
        "yaw_mode",
        "fixed_yaw_deg",
        "takeoff_alt_m",
        "send_rate_hz",
        "mask",
        "land_after",
        "loiter_after_takeoff_sec",
        "loiter_before_land_sec",
        "wp_timeout",
        "wpnav_flyby_count",
    ]
    missing = [k for k in MANDATORY_KEYS if k not in params]
    if missing:
        raise ValueError(
            f"必須キーが不足しています: {', '.join(missing)}"
        )

    # center には latitude / longitude が必須
    center = params["center"]
    for sub_key in ("latitude", "longitude"):
        if sub_key not in center:
            raise ValueError(
                f"center.{sub_key} が不足しています"
            )

    _validate_params(params)
    return params


# =====================================================================
#  SquareFlightController - メインコントローラークラス（WPNAV 互換版）
# =====================================================================

class SquareFlightController:
    """
    Guidedモードで四角形飛行を実行するコントローラークラス（WPNAV 互換版）。

    内部状態遷移:
        WAITING_ARM → TAKEOFF → SQUARE_START → SQUARE_FLYING
        → SQUARE_COMPLETE → LANDING → COMPLETE

    スレッド構成:
        - メインスレッド: 飛行シーケンス制御
        - モニタースレッド: HEARTBEAT/GPS監視、ディスアーム検出
        - レコードスレッド: CSVデータ記録

    execute_square_flight() のみ WPNAV 方式（MAV_CMD_NAV_WAYPOINT 単発送信 +
    到着確認 → stop_at_vertex）に書き換え。その他のメソッドは square_flight.py を踏襲。
    """

    # ── 状態定数 ──
    STATE_WAITING_ARM = "WAITING_ARM"
    STATE_TAKEOFF = "TAKEOFF"
    STATE_SQUARE_START = "SQUARE_START"
    STATE_SQUARE_FLYING = "SQUARE_FLYING"
    STATE_SQUARE_COMPLETE = "SQUARE_COMPLETE"
    STATE_LANDING = "LANDING"
    STATE_COMPLETE = "COMPLETE"

    # シリアル接続設定（square_flight.py のパターンを踏襲）
    SERIAL_DEVICE = '/dev/ttyAMA0'
    SERIAL_BAUD = 1000000

    # CSV保存ディレクトリ（square_flight.py と同様）
    CSV_DIR = Path.home() / "LOGS_Pixhawk6c"



    def __init__(self, param_path):
        """
        コンストラクタ。
        JSONパラメータ読み込み → バリデーション → 頂点計算。

        Args:
            param_path: JSONパラメータファイルのパス
        """
        # ── パラメータ読み込みと検証 ──
        self.params = load_params(param_path)
        print("✓ パラメータ読み込み完了")
        self._print_params()

        # ── 矩形頂点の生成 ──
        self.vertices = generate_square_vertices(
            half_width=self.params["half_width_m"],
            half_height=self.params["half_height_m"],
            direction=self.params["direction"],
        )
        print(f"✓ 矩形頂点生成完了: {len(self.vertices)}点")
        for i, (x, y) in enumerate(self.vertices):
            print(f"    V{i}: ({x:+.3f}, {y:+.3f})m")

        # ── MAVLink接続オブジェクト ──
        self.master = None

        # ── 実行制御フラグ ──
        self._running = False

        # ── スレッド ──
        self._monitor_thread = None
        self._record_thread = None

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

        # ── FCから読み取るパラメータ（connect() で設定）──
        self._fc_wpnav_radius = None    # WPNAV_RADIUS [m]

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
        print(f"  周回数: {p['num_laps']} | 方向: {p['direction']}")
        print(f"  頂点停止: {p['stop_at_vertex_sec']}秒 | 送信レート: {p['send_rate_hz']}Hz")
        print(f"  ヨーモード: {p['yaw_mode']} | land_after: {p['land_after']}")
        print(f"  mode: WPNAV | WPNAV_RADIUS: (connect後にFCから読み取り)")

    # ──────────────────────────────────────────
    #  FCパラメータ読み取り
    # ──────────────────────────────────────────

    def _read_fc_param(self, param_name, default=None, timeout=3.0):
        """
        FCから単一パラメータを PARAM_REQUEST_READ で読み取る。

        Args:
            param_name: パラメータ名（例: "WPNAV_RADIUS"）
            default: 読み取り失敗時のデフォルト値
            timeout: タイムアウト [秒]

        Returns:
            パラメータ値（float）
        """
        if self.master is None:
            print(f"  ⚠ MAVLink未接続のため {param_name} を読めません")
            return default

        # バッファ内の古いメッセージをクリア
        while True:
            msg = self.master.recv_match(blocking=False)
            if msg is None:
                break

        # PARAM_REQUEST_READ 送信
        self.master.mav.param_request_read_send(
            self.master.target_system,
            self.master.target_component,
            param_name.encode('utf-8'),
            -1,
        )

        # PARAM_VALUE 応答を待機
        start = time.time()
        while time.time() - start < timeout:
            msg = self.master.recv_match(
                type='PARAM_VALUE', blocking=True, timeout=0.1
            )
            if msg is None:
                continue
            msg_dict = msg.to_dict()
            msg_param_id = msg_dict.get('param_id', '').rstrip('\x00')
            if msg_param_id == param_name:
                return float(msg_dict.get('param_value', default))

        print(f"  ⚠ {param_name} 読み取りタイムアウト（{timeout}秒）→ default={default}")
        return default

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
        print(f"[INFO] target_system={self.master.target_system}, "
              f"target_component={self.master.target_component}")
        print(f"[INFO] メッセージレート設定: "
              f"GPS_RAW_INT(24), GLOBAL_POSITION_INT(33), ATTITUDE(30) → 5Hz")

        # ── FCパラメータ読み取り ──
        print("\n--- FCパラメータ読み取り ---")
        self._fc_wpnav_radius = self._read_fc_param("WPNAV_RADIUS", default=0.1)
        print(f"  WPNAV_RADIUS = {self._fc_wpnav_radius}m")

        # スレッド起動
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
    #  モニタースレッド（square_flight.py のパターンを踏襲）
    # ──────────────────────────────────────────

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

            # GPS デバッグ: 約1秒毎に受信ログ
            if self._monitor_pos_count % 5 == 0:
                print(
                    f"[GPS] lat={lat:.7f} lon={lon:.7f}"
                    f" rel_alt={alt:.2f}m raw={msg.relative_alt}"
                )

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
    #  セットポイント送信（move_to_start_vertex / return_to_takeoff / stop_at_vertex 用）
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
    #  WPNAV ウェイポイント送信（execute_square_flight 用）
    #  circle_flight_wpnav.py から流用
    # ──────────────────────────────────────────

    def _send_wpnav_waypoint(self, lat_deg, lon_deg, alt_m, yaw_deg):
        """
        MAV_CMD_NAV_WAYPOINT でウェイポイントを1回送信する。

        ArduPilot Guidedモードでは NAV_WAYPOINT を受信すると
        その位置へ自律航行する。到着判定は呼び出し側で行う。

        Args:
            lat_deg: 目標緯度 [deg]
            lon_deg: 目標経度 [deg]
            alt_m: 目標高度（相対高度）[m]
            yaw_deg: 目標ヨー角 [deg]（北=0deg, 東=90deg）
        """
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_WAYPOINT,
            0,                          # confirmation
            0,                          # param1: Hold time [s]（0 = 即時次WPへ）
            self._fc_wpnav_radius,          # param2: Acceptance radius [m]
            0,                          # param3: Pass radius（0 = 通過）
            yaw_deg,                    # param4: Desired yaw [deg]
            lat_deg,                    # param5: Latitude [deg]
            lon_deg,                    # param6: Longitude [deg]
            alt_m,                      # param7: Altitude [m]
        )

        # ログ用に目標位置をローカル座標で更新
        tx, ty, _ = gps_to_local_xyz(
            lat_deg, lon_deg, alt_m,
            self._ref_lat, self._ref_lon, self._ref_alt,
        )
        with self._io_lock:
            self._target.update({'x': tx, 'y': ty, 'z': alt_m})

    # ──────────────────────────────────────────
    #  ヨー角計算（square_flight.py と同一）
    # ──────────────────────────────────────────

    def _calc_yaw(self, v_start, v_end):
        """
        ヨー角を yaw_mode に応じて計算する。

        Args:
            v_start: 始点のローカル座標 (x, y)
            v_end: 終点のローカル座標 (x, y)

        Returns:
            yaw_deg: ヨー角 [deg]（北=0, 東=90）
        """
        yaw_mode = self.params["yaw_mode"]

        if yaw_mode == "fixed":
            return self.params["fixed_yaw_deg"]

        elif yaw_mode == "edge":
            # エッジの進行方向を向く
            dx = v_end[0] - v_start[0]  # 東方向
            dy = v_end[1] - v_start[1]  # 北方向
            # atan2(dx, dy): 北=0, 東=90
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
        最初の頂点 V0 へ移動する。

        - 目標高度 altitude_m に到達するまで待機
        - 頂点V0から閾値以内に近づくまで待機
        - 到達前に定期的にセットポイントを再送信（3秒間隔）

        Returns:
            bool: 到達成功時 True
        """
        print(
            f"\n--- [{self.STATE_SQUARE_START}]"
            " 矩形開始位置（V0）へ移動 ---"
        )
        self._state = self.STATE_SQUARE_START

        # V0 のローカル座標をGPSに変換
        v0_x, v0_y = self.vertices[0]
        lat_v0, lon_v0, _ = local_xyz_to_gps(
            v0_x, v0_y, self.params["altitude_m"],
            self._ref_lat, self._ref_lon, self._ref_alt,
        )

        yaw = self._calc_yaw((0, 0), (v0_x, v0_y))
        self._send_setpoint(lat_v0, lon_v0, self.params["altitude_m"], yaw)
        print(
            f"  目標V0: lat={lat_v0:.7f}, lon={lon_v0:.7f},"
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
            if not alt_reached and z >= target_alt * 0.9:
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
    #  四角形飛行 コアメソッド（WPNAV 方式: コア変更点）
    # ──────────────────────────────────────────

    def stop_at_vertex(self, vertex, duration):
        """
        頂点で指定時間停止（ホバリング）する。

        頂点の座標を send_rate_hz で繰り返し set_position_target_global_int 送信し、
        ArduPilot の位置ホバリング制御に任せる。

        Args:
            vertex: 頂点のローカル座標 (x, y)
            duration: 停止時間 [秒]

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

        print(f"    頂点停止 {duration:.1f}秒...")

        stop_start = time.time()
        while time.time() - stop_start < duration:
            if not self._running:
                return False
            self._send_setpoint(lat, lon, altitude, yaw)
            time.sleep(dt)

        print(f"    ✓ 頂点停止完了")
        return True

    def execute_square_flight(self):
        """
        四角形飛行のメインループを WPNAV 方式で実行する。

        旧方式（square_flight.py）:
            10Hz で set_position_target_global_int を連続送信し、
            ドローンに連続軌道を追従させる。

        新方式（本メソッド）:
            4頂点を MAV_CMD_NAV_WAYPOINT で1回ずつ送信し、
            到着（距離 < WPNAV_RADIUS=10cm）または通過（fly-by）を
            確認した後に stop_at_vertex() で待機、次の頂点へ進む。

        到着判定:
            1. 距離 < WPNAV_RADIUS (10cm) → 到着
            2. 距離が増加に転じたら（fly-by）→ 通過とみなす
            3. タイムアウト（WP_TIMEOUT）→ 続行

        Returns:
            bool: 飛行成功時 True
        """
        print(
            f"\n--- [{self.STATE_SQUARE_FLYING}] 四角形飛行実行 (WPNAV) ---"
        )
        self._state = self.STATE_SQUARE_FLYING

        num_laps = self.params["num_laps"]
        stop_time = self.params["stop_at_vertex_sec"]
        direction = self.params["direction"]
        altitude = self.params["altitude_m"]

        # 総移動距離と推定時間の計算
        total_edge_length = 0.0
        for i in range(4):
            v_start = self.vertices[i]
            v_end = self.vertices[(i + 1) % 4]
            dx = v_end[0] - v_start[0]
            dy = v_end[1] - v_start[1]
            total_edge_length += math.sqrt(dx**2 + dy**2)

        estimated_lap_time = (
            total_edge_length / self.params["speed_m_s"] + stop_time * 4
        )
        print(f"  矩形サイズ: {self.params['half_width_m']*2}m x {self.params['half_height_m']*2}m")
        print(f"  周回数: {num_laps} | 方向: {direction}")
        print(f"  速度: {self.params['speed_m_s']}m/s | 頂点停止: {stop_time}秒")
        print(f"  mode: WPNAV | WPNAV_RADIUS: {self._fc_wpnav_radius}m")
        print(f"  推定1周時間: {estimated_lap_time:.1f}秒")
        print(f"  推定総時間: {estimated_lap_time * num_laps:.1f}秒")

        flight_start = time.time()
        wp_send_count = 0

        for lap in range(num_laps):
            if not self._running:
                print("⚠ 飛行中断")
                return False

            print(f"\n  --- 周回 {lap + 1}/{num_laps} ---")

            for i in range(4):
                if not self._running:
                    print("⚠ 飛行中断")
                    return False

                # 現在の頂点と次の頂点（ヨー角計算用）
                v_target = self.vertices[i]           # 目標頂点
                v_next = self.vertices[(i + 1) % 4]   # 次の頂点

                # ローカル座標 → GPS座標
                lat_wp, lon_wp, _ = local_xyz_to_gps(
                    v_target[0], v_target[1], altitude,
                    self._ref_lat, self._ref_lon, self._ref_alt,
                )

                # ヨー角計算（現在の頂点 → 次の頂点への進行方向）
                yaw = self._calc_yaw(v_target, v_next)

                # ── WPNAVウェイポイントを1回送信 ──
                print(
                    f"    WP[V{i}] → "
                    f"lat={lat_wp:.7f} lon={lon_wp:.7f}"
                    f" yaw={yaw:.1f}deg"
                )
                self._send_wpnav_waypoint(lat_wp, lon_wp, altitude, yaw)
                wp_send_count += 1

                # ── 到着判定ループ ──
                wp_start = time.time()
                min_dist = float('inf')
                increasing_count = 0
                arrived = False

                while time.time() - wp_start < self.params["wp_timeout"]:
                    if not self._running:
                        print("⚠ 飛行中断")
                        return False

                    with self._io_lock:
                        gx = self._gps_now['x']
                        gy = self._gps_now['y']

                    # 目標頂点のローカル座標を計算
                    wx, wy, _ = gps_to_local_xyz(
                        lat_wp, lon_wp, altitude,
                        self._ref_lat, self._ref_lon, self._ref_alt,
                    )
                    dist = math.sqrt((gx - wx) ** 2 + (gy - wy) ** 2)

                    # 最小距離を更新
                    if dist < min_dist:
                        min_dist = dist

                    # 判定1: 距離 < WPNAV_RADIUS → 到着
                    if dist < self._fc_wpnav_radius:
                        arrived = True
                        break

                    # 判定2: 距離が増加に転じた → 通過（fly-by）
                    if dist > min_dist + 0.02:
                        increasing_count += 1
                    else:
                        increasing_count = 0

                    if increasing_count >= self.params["wpnav_flyby_count"]:
                        arrived = True
                        break

                    time.sleep(0.1)

                # 到着確認
                reason = "arrived" if min_dist < self._fc_wpnav_radius else "fly-by"
                elapsed_wp = time.time() - wp_start
                print(
                    f"      V{i} dist={min_dist:.3f}m"
                    f" {'✓' if arrived else '⚠ TIMEOUT'}"
                    f" ({elapsed_wp:.1f}s, {reason})"
                )

                if not arrived:
                    print(
                        f"⚠ V{i} タイムアウト"
                        f"（{self.params['wp_timeout']}秒, min_dist={min_dist:.3f}m）— 続行"
                    )

                # 最終エッジかつ最終周回では停止しない
                is_last = (lap == num_laps - 1) and (i == 3)
                if not is_last:
                    if not self.stop_at_vertex(v_target, stop_time):
                        return False
                    print(f"  ✓ 頂点 V{i} 到達")
                else:
                    print(f"  ✓ 頂点 V{i} 到着（最終頂点、停止なし）")

        total_elapsed = time.time() - flight_start
        print(f"\n✓ 全周回完了（{total_elapsed:.1f}秒, {wp_send_count}回送信）")
        self._state = self.STATE_SQUARE_COMPLETE
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
        print(f"\n--- [{self.STATE_SQUARE_COMPLETE}] 離陸地点上空へ帰還 ---")

        altitude = self.params["altitude_m"]  # 現在の飛行高度を維持

        print(f"  目標: lat={self._takeoff_lat:.7f}, lon={self._takeoff_lon:.7f}, alt={altitude}m")

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
                print(f"⚠ タイムアウト: 離陸地点帰還（{timeout}秒, 距離={dist:.2f}m）")
                return False

            # 3秒毎にセットポイント再送
            if int(time.time() - start) % 3 == 0:
                self._send_setpoint(self._takeoff_lat, self._takeoff_lon, altitude,
                                    self.params["fixed_yaw_deg"])

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
            5. 四角形飛行実行（WPNAV方式）
            6. 離陸地点帰還 → 2秒待機 → 着陸
        """
        print("=" * 60)
        print("  ArduPilot 四角形飛行制御 - Square Flight (WPNAV)")
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

            # 4. 矩形開始位置（V0）へ移動
            if not self.move_to_start_vertex():
                print("\n✗ 開始位置移動失敗 → 着陸試行")
                self.land()
                return

            # 5. 四角形飛行実行（WPNAV方式）
            if not self.execute_square_flight():
                print("\n✗ 四角形飛行中断 → 着陸試行")
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

        保存先: ~/LOGS_Pixhawk6c/{YYYYMMDD_HHMMSS}_square_wpnav.csv
        フォーマット: Time, GPS_X, GPS_Y, GPS_Z, Target_X, Target_Y, Target_Z
        """
        if not self._data_records:
            print("⚠ 記録データなし")
            return

        now = datetime.datetime.now(
            pytz.timezone("Asia/Tokyo")
        ).strftime("%Y%m%d_%H%M%S")
        path = self.CSV_DIR / f"{now}_square_wpnav.csv"

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
        python square_flight_wpnav.py [パラメータファイルパス]

    引数なしの場合、スクリプトと同じディレクトリの
    square_params.json を読み込む。
    """
    # パラメータファイルパスの決定
    if len(sys.argv) > 1:
        param_path = Path(sys.argv[1])
    else:
        param_path = Path(__file__).parent / "square_params.json"

    if not param_path.exists():
        print(f"✗ パラメータファイルが見つかりません: {param_path}")
        print("  使用法: python square_flight_wpnav.py [params.json]")
        sys.exit(1)

    print(f"パラメータファイル: {param_path}")

    # コントローラー生成（ここでパラメータ読み込みとバリデーション）
    try:
        controller = SquareFlightController(str(param_path))
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
