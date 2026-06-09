# 円飛行スクリプト 計画ドキュメント

> **作成日**: 2026-06-07  
> **対象ディレクトリ**: `/home/taki/Mavlink_raspi/EKF/`  
> **参考コード**: `/home/taki/Mavlink_raspi/No1/up_to_down3.py`

---

## 1. 概要

Guidedモードで、指定された中心座標を中心に指定半径の円を描いて飛行するPythonスクリプトの設計計画。
JSONパラメータファイルから設定を読み込み、離陸 → 円飛行 → 着陸のシーケンスを自動実行する。

---

## 2. 参考コード解析（up_to_down3.py）

### 2.1 全体構成

| 要素 | 内容 |
|------|------|
| MAVLink接続 | `/dev/ttyAMA0`, 1,000,000bps, RTS/CTS有効 |
| 制御方式 | `set_position_target_global_int_send` による位置制御 |
| 座標変換 | `gps_to_local_xyz()` / `local_xyz_to_gps()` |
| モード | Guidedモード（custom_mode == 4） |
| 離陸 | `MAV_CMD_NAV_TAKEOFF`, 高度 0.5m |
| スレッド構成 | メイン制御 + 状態監視 + CSV記録（3スレッド） |
| CSV記録 | 時刻 / GPS_X, Y, Z / Target_X, Y, Z |
| 安全機能 | ディスアーム検出で `running=False` |
| 送信周期 | 10Hz（`SEND_HZ = 10`） |

### 2.2 キーポイント（新スクリプトに継承すべき要素）

1. **MAVLink接続パターン**: `mavutil.mavlink_connection()` + `wait_heartbeat()`
2. **Guidedモード検出**: `hb.custom_mode == 4`
3. **離陸シーケンス**: モード検出 → 3秒待機 → `MAV_CMD_NAV_TAKEOFF`
4. **setpoint送信**: `set_position_target_global_int_send` + マスク `0x09F8`（位置のみ制御）
5. **座標変換の定数**: 緯度経度→メートル変換に `111319.5 [m/deg]` を使用
6. **エラーハンドリング**: `signal.SIGINT` による安全停止

---

## 3. 円飛行アルゴリズム設計

### 3.1 数学的定式化

#### 3.1.1 円周上のウェイポイント座標計算

中心座標 (緯度 lat_c, 経度 lon_c)、半径 R [m]、ウェイポイント数 N とする。

**Step 1: ローカル座標系での位置計算**

i 番目 (i = 0, 1, ..., N-1) のウェイポイントの角度 θ_i を以下で定義する：

- **時計回り（CW）**: θ_i = -2πi / N
- **反時計回り（CCW）**: θ_i = 2πi / N

ウェイポイントのローカル座標 (x_i, y_i) は、中心を原点として：

```
x_i = R * cos(θ_i)     (東方向)
y_i = R * sin(θ_i)     (北方向)
```

**Step 2: GPS座標への変換**

ローカル座標 → GPS座標の変換式（`local_xyz_to_gps()` と等価）：

```
lat_i = lat_c + y_i / 111319.5
lon_i = lon_c + x_i / (111319.5 * cos(lat_c * π / 180))
```

ここで、
- `111319.5` [m/deg] は赤道における1度あたりの距離（WGS84近似）
- `cos(lat_c)` は緯度に応じた経度方向の縮小率

**Step 3: 高度**

全ウェイポイントで一定高度 H [m]（relative altitude）を使用：

```
alt_i = H
```

#### 3.1.2 タイミング制御

- 一周時間 T [秒]、ウェイポイント数 N
- 各ウェイポイント間の時間間隔: Δt = T / N [秒]
- ウェイポイント i の目標時刻: t_i = i * Δt（t_0 = 0 を円開始時刻とする）

**送信戦略**: 各ウェイポイントに達する前に次のウェイポイントを先行的に送信する。
`set_position_target_global_int_send` は「ここに向かえ」という目標を与えるため、
ArduPilotの位置制御が滑らかな補間を行う。

#### 3.1.3 ヨー角制御

3つのモードをパラメータで選択可能とする：

| モード | 計算式 | 説明 |
|--------|--------|------|
| `tangent` | ψ_i = θ_i + 90° (CCW) / θ_i - 90° (CW) | 進行方向を向く |
| `center` | ψ_i = θ_i + 180° | 円の中心を向く |
| `fixed` | ψ_i = const | 固定ヨー角 |

（角度は北=0°, 東=90°とする。MAVLinkのyawは北基準0ラジアン）

### 3.2 飛行シーケンス（状態遷移）

```
[INIT] → [ARM/Guided待機] → [離陸] → [離陸高度到達]
       → [円中心へ移動] → [円飛行ループ] → [着陸] → [END]
```

#### 状態定義

| 状態 | 内容 |
|------|------|
| `WAITING_ARM` | Guided+Arm待機。Arm検出後、離陸シーケンスへ |
| `TAKEOFF` | 離陸指令送信後、目標高度到達を待機 |
| `CIRCLE_START` | 円周上の最初のウェイポイントへ移動 |
| `CIRCLE_FLYING` | 円周上のウェイポイントを順次送信。タイマーで切り替え |
| `CIRCLE_COMPLETE` | 全周回完了 |
| `LANDING` | 着陸指令（MAV_CMD_NAV_LAND） |
| `COMPLETE` | 終了 |

### 3.3 円中心への移動戦略

離陸後、最初に円周上の最初のウェイポイント（θ_0）に移動してから円飛行を開始する。


---

## 4. JSONパラメータスキーマ設計

### 4.1 設定ファイル仕様

ファイル名: `circle_params.json`（デフォルト）

```jsonc
{
  // ===== 円飛行設定 =====
  "center": {
    "latitude": 36.0757800,        // 円の中心緯度 [deg]
    "longitude": 136.2132900       // 円の中心経度 [deg]
  },
  "radius_m": 5.0,                 // 円の半径 [m]（> 0、例: 0.3〜）
  "altitude_m": 2.0,               // 飛行高度 [m]（>= 0.5）
  "lap_time_sec": 30.0,            // 一周にかかる時間 [秒]（>= 5.0）
  "num_laps": 1,                   // 周回数（>= 1）

  // ===== 軌道生成パラメータ =====
  "num_waypoints": 36,             // 一周のウェイポイント数（>= 8, <= 360）
  "direction": "CW",               // 回転方向: "CW"（時計回り）or "CCW"

  // ===== ヨー角設定 =====
  "yaw_mode": "tangent",           // "tangent" | "center" | "fixed"
  "fixed_yaw_deg": 0.0,           // yaw_mode="fixed"時の固定ヨー角 [deg]

  // ===== 離陸設定 =====
  "takeoff_alt_m": 0.5,           // 離陸高度 [m]

  // ===== 通信設定 =====
  "send_rate_hz": 10,             // 送信レート [Hz]
  "mask": "0x09F8",               // set_position_target_global_intのマスク

  // ===== 安全設定 =====
  "loiter_after_takeoff_sec": 3.0, // 離陸後の安定待機時間 [秒]
  "loiter_before_land_sec": 3.0    // 着陸前の安定待機時間 [秒]
}
```

### 4.2 バリデーションルール

| パラメータ | 制約 |
|------------|------|
| `center.latitude` | -90.0 <= lat <= 90.0 |
| `center.longitude` | -180.0 <= lon <= 180.0 |
| `radius_m` | > 0.0 |
| `altitude_m` | >= 0.5 |
| `lap_time_sec` | >= 5.0 |
| `num_laps` | >= 1 |
| `num_waypoints` | >= 8, <= 360 |
| `direction` | "CW" or "CCW" |
| `yaw_mode` | "tangent" / "center" / "fixed" |
| `fixed_yaw_deg` | 0.0 <= yaw <= 360.0 |


---

## 5. 全体アーキテクチャ設計

### 5.1 クラス構成

```
circle_flight.py
├── CircleFlightController （メインコントローラークラス）
│   ├── __init__(self, param_path: str)
│   ├── connect(self) -> bool
│   ├── wait_for_guided_arm(self) -> bool
│   ├── takeoff(self) -> bool
│   ├── move_to_start_position(self) -> bool
│   ├── execute_circle_flight(self) -> bool
│   ├── land(self) -> bool
│   ├── run(self) -> None
│   └── cleanup(self) -> None
│
├── ヘルパー関数（モジュールレベル）
│   ├── gps_to_local_xyz(lat, lon, alt) -> (x, y, z)
│   ├── local_xyz_to_gps(x, y, z) -> (lat, lon, alt)
│   ├── generate_circle_waypoints(center, radius, n, direction) -> list
│   └── load_params(path) -> dict
```

### 5.2 スレッド構成

| スレッド | 役割 | 起動タイミング | 停止条件 |
|----------|------|---------------|----------|
| **メイン** | 飛行シーケンス制御 | スクリプト開始 | シーケンス完了 |
| **モニター** | HEARTBEAT/GPS監視、ディスアーム検出 | `connect()` 後 | `self._running=False` |
| **レコード** | CSVデータ記録 | `connect()` 後 | `self._running=False` |

### 5.3 データフロー

```
[JSONパラメータ]
    ↓ load_params()
[パラメータ検証]
    ↓
[ウェイポイント生成] generate_circle_waypoints()
    ↓
[MAVLink接続] connect()
    ↓
[飛行シーケンス]
    ├── wait_for_guided_arm()
    ├── takeoff()
    ├── move_to_start_position()
    ├── execute_circle_flight()
    │   └── [ウェイポイントリストを順次 send_setpoint()]
    └── land()
    ↓
[CSV保存] save_csv()
    ↓
[クリーンアップ] cleanup()
```

### 5.4 CSV記録仕様（参考コードと同一フォーマット）

```csv
Time, GPS_X, GPS_Y, GPS_Z, Target_X, Target_Y, Target_Z
```

- 保存先: `~/LOGS_Pixhawk6c/`
- ファイル名: `{YYYYMMDD_HHMMSS}_circle.csv`


---

## 6. 詳細クラス・関数設計

### 6.1 CircleFlightController

```python
class CircleFlightController:
    """
    Guidedモードで円飛行を実行するコントローラークラス。
    JSONパラメータファイルから設定を読み込み、
    離陸→円飛行→着陸のシーケンスを自動実行する。
    """

    # ── プロパティ ──
    # self.params: dict        # JSONから読み込んだパラメータ
    # self.waypoints: list     # 円周上のGPS座標リスト [(lat, lon, alt, yaw), ...]
    # self.master: mavutil     # MAVLink接続オブジェクト
    # self._running: bool      # 全スレッドの実行フラグ
    # self._gps_now: dict      # 現在のGPSローカル座標 {'x','y','z'}
    # self._target: dict       # 現在の目標座標 {'x','y','z'}
    # self._data_records: list # CSV記録用データ
    # self._io_lock: Lock      # スレッド間排他ロック
    # self._state: str         # 現在の状態（状態遷移用）
```

#### 6.1.1 `__init__(self, param_path)`
- JSON読み込み → `load_params()`
- パラメータ検証
- ウェイポイント生成 → `generate_circle_waypoints()`
- フラグ・ロック初期化

#### 6.1.2 `connect(self)`
- `/dev/ttyAMA0`, 1,000,000bps, RTS/CTS
- `wait_heartbeat()`
- メッセージレート設定（GLOBAL_POSITION_INT, ATTITUDE, HEARTBEAT）
- モニター/レコードスレッド起動

#### 6.1.3 `wait_for_guided_arm(self)`
- `HEARTBEAT` 受信ループ
- `custom_mode == 4`（Guided）かつ `base_mode & MAV_MODE_FLAG_SAFETY_ARMED` 待機
- タイムアウト: 60秒

#### 6.1.4 `takeoff(self)`
- Guided+Arm 検出後、`loiter_after_takeoff_sec` 秒待機
- `MAV_CMD_NAV_TAKEOFF` 送信（`takeoff_alt_m`）
- `GLOBAL_POSITION_INT` で高度が `takeoff_alt_m * 0.9` に達するのを待機
- タイムアウト: 30秒

#### 6.1.5 `move_to_start_position(self)`
- 最初のウェイポイント座標を送信
- 現在位置が閾値以内に近づくまで待機
- 閾値: 目標から半径の20%以内

#### 6.1.6 `execute_circle_flight(self)`
- `num_laps` 回ループ
  - 各ラップで全ウェイポイントを順次処理
  - `time.sleep(dt)` 間隔で `send_setpoint()` 送信
- dt = lap_time_sec / num_waypoints
- 各ラップの開始/終了をログ出力

#### 6.1.7 `land(self)`
- 円飛行完了後、`loiter_before_land_sec` 秒待機
- `MAV_CMD_NAV_LAND` 送信
- ディスアーム検出まで待機（タイムアウト: 60秒）

#### 6.1.8 `run(self)`
```python
def run(self):
    try:
        self.connect()
        self.wait_for_guided_arm()
        self.takeoff()
        self.move_to_start_position()
        self.execute_circle_flight()
        self.land()
    except Exception as e:
        print(f"エラー: {e}")
    finally:
        self.cleanup()
```

#### 6.1.9 `cleanup(self)`
- `self._running = False`
- スレッド終了待機
- CSV保存


### 6.2 ヘルパー関数

#### 6.2.1 `gps_to_local_xyz(lat, lon, alt, ref_lat, ref_lon, ref_alt)`

```python
def gps_to_local_xyz(lat, lon, alt, ref_lat, ref_lon, ref_alt):
    """
    GPS座標を基準点からのローカル直交座標に変換する。
    Returns: (dx, dy, dz) [m] 東, 北, 上
    """
    cos_lat = math.cos(math.radians(ref_lat))
    dx = (lon - ref_lon) * 111319.5 * cos_lat
    dy = (lat - ref_lat) * 111319.5
    dz = alt - ref_alt
    return dx, dy, dz
```

#### 6.2.2 `local_xyz_to_gps(x, y, z, ref_lat, ref_lon, ref_alt)`

```python
def local_xyz_to_gps(x, y, z, ref_lat, ref_lon, ref_alt):
    """
    ローカル直交座標をGPS座標に変換する。
    Returns: (lat, lon, alt) [deg, deg, m]
    """
    cos_lat = math.cos(math.radians(ref_lat))
    lat = ref_lat + y / 111319.5
    lon = ref_lon + x / (111319.5 * cos_lat)
    alt = ref_alt + z
    return lat, lon, alt
```

#### 6.2.3 `generate_circle_waypoints()`

```python
def generate_circle_waypoints(center, radius, n, direction,
                              altitude, yaw_mode, fixed_yaw_deg):
    """
    円周上のウェイポイントリストを生成する。
    Returns: [(lat, lon, alt, yaw_deg), ...]
    """
    waypoints = []
    ref_lat, ref_lon = center["latitude"], center["longitude"]

    for i in range(n):
        if direction == "CW":
            theta = -2.0 * math.pi * i / n
        else:  # CCW
            theta = 2.0 * math.pi * i / n

        x = radius * math.cos(theta)
        y = radius * math.sin(theta)

        lat_wp, lon_wp, _ = local_xyz_to_gps(
            x, y, 0.0, ref_lat, ref_lon, 0.0)

        if yaw_mode == "tangent":
            yaw = ((math.degrees(theta) + 90) % 360
                   if direction == "CCW"
                   else (math.degrees(theta) - 90) % 360)
        elif yaw_mode == "center":
            yaw = (math.degrees(theta) + 180) % 360
        else:
            yaw = fixed_yaw_deg

        waypoints.append((lat_wp, lon_wp, altitude, yaw))

    return waypoints
```

#### 6.2.4 `load_params(path)`

```python
def load_params(path):
    """
    JSONパラメータファイルを読み込み、バリデーションを行う。
    Raises: FileNotFoundError, ValueError
    """
    import json
    with open(path, 'r') as f:
        params = json.load(f)
    # バリデーション処理（4.2節のルールに従う）
    _validate_params(params)
    return params
```

---

## 7. 送信関数設計

### 7.1 `send_setpoint()`

```python
def send_setpoint(self, lat_i, lon_i, alt, yaw_deg):
    """
    位置目標を送信する。
    Args:
        lat_i: 緯度 [deg * 1e7]
        lon_i: 経度 [deg * 1e7]
        alt: 高度 [m] (relative altitude)
        yaw_deg: 目標ヨー角 [deg]
    """
    mask = int(self.params["mask"], 16)  # 0x09F8
    self.master.mav.set_position_target_global_int_send(
        0,                          # time_boot_ms
        self.master.target_system,
        self.master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        mask,
        lat_i, lon_i, alt,          # 位置
        0, 0, 0,                    # 速度（制御しない）
        0, 0, 0,                    # 加速度（制御しない）
        math.radians(yaw_deg),      # ヨー角
        0                           # ヨーレート（制御しない）
    )
```

---


## 8. 安全機能設計

### 8.1 ディスアーム検出

モニタースレッドでHEARTBEATを監視。
`base_mode & MAV_MODE_FLAG_SAFETY_ARMED == 0` 検出時：
1. `self._running = False` に設定（全スレッド停止）
2. CSV記録を即座に保存
3. プログラム終了

### 8.2 タイムアウト一覧

| 操作 | タイムアウト | 動作 |
|------|-------------|------|
| MAVLink接続 | 10秒 | エラー出力 → 終了 |
| Arm/Guided待機 | 60秒 | 警告出力 → 終了 |
| 離陸高度到達 | 30秒 | エラー → 着陸試行 |
| 円飛行（全体） | `lap_time_sec * num_laps * 2` | エラー → 着陸試行 |
| 着陸完了 | 60秒 | エラー出力 → 終了 |

### 8.3 シグナルハンドリング

```python
signal.signal(signal.SIGINT,  lambda s, f: self._handle_interrupt())
signal.signal(signal.SIGTERM, lambda s, f: self._handle_interrupt())
```

割り込み時に `self._running = False` を設定し、可能な限り着陸試行後CSV保存。

### 8.4 事前チェック

```python
def preflight_check(self) -> bool:
    """飛行前の安全チェック"""
    # - MAVLink接続確認
    # - パラメータ妥当性確認
    # - GPS fix 確認（オプション）
```

---

## 9. 実装ファイル構成

```
/home/taki/Mavlink_raspi/EKF/
├── circle_flight_plan.md          # 本計画ドキュメント
├── circle_flight.py               # メインスクリプト（実装予定）
├── circle_params.json             # パラメータ設定ファイル（実装予定）
└── circle_params_template.json     # パラメータテンプレート（実装予定）
```

---

## 10. 操作手順（Operation Manual）

### 10.1 前提条件

#### ハードウェア要件

| 項目 | 要件 |
|------|------|
| コンパニオンコンピュータ | Raspberry Pi（シリアルポート `/dev/ttyAMA0` に接続） |
| フライトコントローラ | Pixhawk 系（ArduPilotファームウェア、Guidedモード対応） |
| GPS環境 | GPS受信可能な環境（屋外またはSITL） |
| 接続 | UARTシリアル接続（RTS/CTS有効、1,000,000bps） |

#### ソフトウェア要件

| 項目 | バージョン/備考 |
|------|----------------|
| Python | 3.7 以上 |
| pymavlink | MAVLink通信用 |
| pytz | タイムゾーン処理用（Asia/Tokyo） |

#### 機体要件

- Arm可能な状態（安全スイッチ解除済み、RC受信機接続）
- GPS fix 取得済み
- Guidedモードが有効なファームウェア

---

### 10.2 事前準備

#### 10.2.1 パラメータファイル（circle_params.json）の編集

スクリプトと同じディレクトリに `circle_params.json` を作成し、以下の形式で編集する：

```json
{
  "center": {
    "latitude": 36.0757800,
    "longitude": 136.2132900
  },
  "radius_m": 5.0,
  "altitude_m": 2.0,
  "lap_time_sec": 30.0,
  "num_laps": 1,
  "num_waypoints": 36,
  "direction": "CW",
  "yaw_mode": "tangent",
  "fixed_yaw_deg": 0.0,
  "takeoff_alt_m": 0.5,
  "send_rate_hz": 10,
  "mask": "0x09F8",
  "land_after": true,
  "loiter_after_takeoff_sec": 3.0,
  "loiter_before_land_sec": 3.0
}
```

**パラメータ一覧と推奨値**：

| パラメータ | 型 | デフォルト | 制約 | 説明 |
|-----------|------|-----------|------|------|
| `center.latitude` | float | — | -90.0 〜 90.0 | 円の中心緯度 [deg] |
| `center.longitude` | float | — | -180.0 〜 180.0 | 円の中心経度 [deg] |
| `radius_m` | float | 5.0 | > 0.0 | 円の半径 [m]（屋外推奨 2m以上、屋内 0.3m〜） |
| `altitude_m` | float | 2.0 | >= 0.5 | 飛行高度（相対高度）[m] |
| `lap_time_sec` | float | 30.0 | >= 5.0 | 一周にかかる時間 [秒] |
| `num_laps` | int | 1 | >= 1 | 周回数 |
| `num_waypoints` | int | 36 | 8 〜 360 | 一周のウェイポイント数（多いほど滑らか） |
| `direction` | string | "CW" | "CW" または "CCW" | 回転方向（CW=時計回り, CCW=反時計回り） |
| `yaw_mode` | string | "tangent" | "tangent" / "center" / "fixed" | ヨー角制御モード |
| `fixed_yaw_deg` | float | 0.0 | 0.0 〜 360.0 | yaw_mode="fixed"時の固定ヨー角 [deg] |
| `takeoff_alt_m` | float | 0.5 | — | 離陸高度 [m] |
| `send_rate_hz` | int | 10 | >= 1 | セットポイント送信レート [Hz]（推奨: 5〜10） |
| `mask` | string | "0x09F8" | — | 位置制御マスク（位置+ヨー角のみ制御） |
| `land_after` | bool | true | true / false | 円飛行完了後に着陸するか（falseの場合はその場で停止） |
| `loiter_after_takeoff_sec` | float | 3.0 | — | 離陸後の安定待機時間 [秒] |
| `loiter_before_land_sec` | float | 3.0 | — | 着陸前の安定待機時間 [秒] |

#### 10.2.2 機体の設置

1. 中心座標から十分離れた安全な場所に機体を設置する（円の半径以上離すことを推奨）
2. プロペラを装着し、バッテリーを接続する
3. 安全スイッチを解除する
4. フライトコントローラとRaspberry Piをシリアルケーブルで接続する

---

### 10.3 実行方法

#### 10.3.1 基本実行

```bash
# 1. 作業ディレクトリに移動
cd /home/taki/Mavlink_raspi/EKF/

# 2. スクリプト実行（sudoが必要な場合あり）
python3 circle_flight.py [パラメータファイルパス]
```

#### 10.3.2 コマンドライン

```
使用法: python3 circle_flight.py [params.json]
```

- **引数なし**: スクリプトと同じディレクトリの `circle_params.json` を読み込む
- **引数あり**: 指定されたパスのJSONファイルを読み込む

#### 10.3.3 実行例

```bash
# デフォルトの circle_params.json を使用
python3 circle_flight.py

# 別の設定ファイルを指定
python3 circle_flight.py my_circle_params.json
```

---

### 10.4 実行シーケンス（タイムライン）

スクリプト起動後の自動実行シーケンス：

| # | フェーズ | 処理内容 | 所要時間目安 |
|---|---------|---------|------------|
| 1 | パラメータ読み込み | JSONファイル読み込み → バリデーション → ウェイポイント計算 | 即時 |
| 2 | MAVLink接続 | `/dev/ttyAMA0` に接続 → ハートビート待機 → メッセージレート設定 → モニター/レコードスレッド起動 | 〜10秒 |
| 3 | Guided+Arm待機 | HEARTBEAT監視、Guidedモード（custom_mode=4）+ Armフラグ検出待ち | 手動操作次第（タイムアウト60秒） |
| 4 | 離陸 | `loiter_after_takeoff_sec` 秒待機 → `MAV_CMD_NAV_TAKEOFF` 送信（`takeoff_alt_m`）→ 高度到達待ち | 〜30秒 |
| 5 | 目標高度へ上昇＆円開始位置へ移動 | `altitude_m` への上昇 + 円周上の最初のウェイポイントへ移動 | 〜90秒 |
| 6 | 円飛行実行 | `num_laps` 周 × `num_waypoints` 点のウェイポイントを順次送信 | `lap_time_sec` × `num_laps` |
| 7 | 着陸 | `loiter_before_land_sec` 秒待機 → `MAV_CMD_NAV_LAND` 送信 → ディスアーム検出待ち | 〜60秒 |
| 8 | CSV保存・終了 | データをCSVに保存 → スレッド終了 | 即時 |

> **注意**: フェーズ3（Guided+Arm待機）では、ユーザーが手動でモード切替とArmを行う必要がある。スクリプトは自動的に検出して次フェーズに進む。

---

### 10.5 実行中の表示メッセージ

スクリプト実行中に表示される主なメッセージとその意味：

| メッセージ例 | 意味 |
|-------------|------|
| `✓ パラメータ読み込み完了` | JSON読み込みとバリデーション成功 |
| `✓ ウェイポイント生成完了: 36点` | 円周上のウェイポイント計算完了 |
| `--- MAVLink接続 ---` | MAVLink接続フェーズ開始 |
| `✓ MAVLink接続完了 (sys=..., ver=...)` | ハートビート受信、接続確立 |
| `✓ モニター/レコードスレッド起動` | バックグラウンド監視スレッド開始 |
| `--- [WAITING_ARM] Guided+Arm待機 ---` | Arm検出待機中 |
| `✓ Guidedモード+Arm検出（X.X秒）` | Arm検出成功、離陸フェーズへ |
| `--- [TAKEOFF] 離陸 ---` | 離陸フェーズ開始 |
| `✓ 離陸指令送信（0.5m）` | MAV_CMD_NAV_TAKEOFF 送信完了 |
| `✓ 離陸高度到達: 0.50m` | 目標離陸高度に到達 |
| `--- [CIRCLE_START] 円開始位置へ移動 ---` | 円の開始位置への移動中 |
| `✓ 目標高度到達: 2.00m` | 円飛行高度に到達 |
| `✓ 開始位置到達: 距離=X.XXm` | 最初のWP付近に到達 |
| `--- [CIRCLE_FLYING] 円飛行実行 ---` | 円飛行ループ開始 |
| `--- 周回 1/3 ---` | ラップ開始 |
| `✓ 周回 1 完了（30.0秒/36送信）` | ラップ完了 |
| `✓ 全周回完了（90.0秒, 108回送信）` | 全ラップ完了 |
| `--- [LANDING] 着陸 ---` | 着陸フェーズ開始 |
| `✓ 着陸指令送信` | MAV_CMD_NAV_LAND 送信完了 |
| `✓ 着陸+ディスアーム完了` | 着陸検出、正常終了 |
| `✓ CSV保存完了: /home/taki/LOGS_Pixhawk6c/20260607_120000_circle.csv（XXX行）` | CSV保存完了 |
| `✓ [COMPLETE] 全シーケンス完了` | 全シーケンス正常終了 |

---

### 10.6 異常時の対応

| 状況 | スクリプトの自動動作 | 推奨される手動対応 |
|------|---------------------|-------------------|
| MAVLink接続失敗（10秒タイムアウト） | エラー出力 → プログラム終了 | ケーブル接続・ボーレートを確認 |
| Arm未検出（60秒タイムアウト） | タイムアウトメッセージ → プログラム終了 | 手動でGuidedモード切替＋Armする |
| 離陸失敗（30秒タイムアウト） | エラー出力 → 着陸試行 → プログラム終了 | RCでマニュアル操作に切替 |
| 開始位置到達タイムアウト（90秒） | 警告出力 → 飛行続行（タイムアウトでも継続） | 飛行に支障なければ継続監視 |
| 飛行中ディスアーム検出 | `⚠ ディスアーム検出 → 安全停止` → CSV保存 → 終了 | 機体の安全を確保 |
| 飛行中モード切替（Guided→他） | `⚠ モード変更検出 → 安全停止` → CSV保存 → 終了 | 意図的な切替か確認 |
| Ctrl+C（SIGINT） | `⚠ 割り込み信号検出 → 安全停止` → 着陸試行 → CSV保存 → 終了 | 必要に応じてRCで直接操作 |
| SIGTERM | SIGINTと同様に安全停止 | 同上 |

---

### 10.7 ログの確認

#### 保存先
```
/home/taki/LOGS_Pixhawk6c/
```

#### ファイル名形式
```
YYYYMMDD_HHMMSS_circle.csv
```
（例: `20260607_120000_circle.csv`、タイムゾーン: Asia/Tokyo）

#### CSVカラム構成

| カラム | 内容 |
|--------|------|
| `Time` | 記録時刻（`time.time()` のUNIX秒） |
| `GPS_X` | 現在GPS位置のローカルX座標 [m]（中心基準、東方向） |
| `GPS_Y` | 現在GPS位置のローカルY座標 [m]（中心基準、北方向） |
| `GPS_Z` | 現在GPS高度 [m]（相対高度） |
| `Target_X` | 目標位置のローカルX座標 [m]（中心基準、東方向） |
| `Target_Y` | 目標位置のローカルY座標 [m]（中心基準、北方向） |
| `Target_Z` | 目標高度 [m] |

#### ログの活用方法

- GPS_X, GPS_Y を散布図または折れ線グラフでプロットし、実際の飛行軌跡が円を描いているか確認できる
- Target_X, Target_Y と GPS_X, GPS_Y を重ねてプロットすることで、目標との追従精度を評価できる
- GPS_Z の時系列推移から高度制御の応答を確認できる

---

## 11. テスト計画

### 11.1 ユニットテスト（机上）

| テスト項目 | 確認内容 |
|------------|---------|
| 座標変換の往復 | gps→local→gps で元の値に戻ること |
| ウェイポイント生成 | 正しい数、正しい座標範囲 |
| パラメータバリデーション | 不正値で例外発生 |

### 11.2 SITLテスト（推奨）

1. SITL（Software In The Loop）でシミュレーション実行
2. 小半径（0.3m〜2m）× 低高度（1m）× 短時間（10秒ラップ）で基本動作確認
3. CSVログを解析し、実際の軌跡が円を描いているか確認

### 11.3 実機テスト（段階的）

1. プロペラなしでMAVLink通信確認
2. 超小半径（0.3m〜1m）× 低高度（0.5m）× 1周 × 屋内
3. 徐々に半径・高度・周回数を拡大

---

## 12. 実装上の注意点

### 12.1 set_position_target_global_int の送信間隔

- ArduPilotは最後に受信したsetpointから約3〜5秒でタイムアウトし、ホバリングに切り替わる
- `send_rate_hz >= 1`（1秒以内に必ず次のsetpointを送信）を必須とする
- 推奨: 5〜10Hz

### 12.2 座標変換の精度

- 平面近似（`111319.5 * cos(lat)`）を使用しているため、半径100m程度までが実用範囲
- 大規模な円の場合はHaversine式など球面三角法を検討

### 12.3 風の影響

- ArduPilotの位置制御が補正するが、半径が小さいほど（<2m）風の影響が顕著になる
- 屋外では半径 2m 以上を推奨（屋内・無風環境では 0.3m も可能）

### 12.4 通信遅延

- `/dev/ttyAMA0`（UART）経由で数ms〜数十msの遅延
- `send_rate_hz` が高すぎると送信キューが詰まる可能性がある
- 推奨: 10Hz

---

## 13. 拡張アイデア（将来）

1. 複数円の連結飛行（8の字飛行など）
2. 高度可変の螺旋飛行
3. リアルタイム位置フィードバックによる軌道補正
4. MQTT等による遠隔パラメータ変更
5. GUIパラメータ設定ツール

---

## 14. まとめ

本ドキュメントでは、Guidedモードで円飛行を実現するスクリプトの設計計画をまとめた。

**主要な設計判断**:
- 参考コード（`up_to_down3.py`）の実績あるMAVLink通信パターンを踏襲
- ウェイポイントは事前に全点を計算し、タイマーで順次送信するシンプルな方式
- 3スレッド構成（制御/監視/記録）で安全面を確保
- JSONパラメータによる柔軟な設定変更

**次のステップ**: `circle_flight.py` の実装
