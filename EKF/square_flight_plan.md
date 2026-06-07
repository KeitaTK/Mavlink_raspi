# 四角形（矩形）飛行スクリプト 技術調査・設計計画

> **作成日**: 2026-06-07  
> **対象ディレクトリ**: `/home/taki/Mavlink_raspi/EKF/`  
> **参考コード**: `circle_flight.py`, `circle_flight_plan.md`  
> **調査対象**: ArduPilot GUIDED モードの速度制御, SET_POSITION_TARGET_GLOBAL_INT type_mask, MAV_CMD_DO_CHANGE_SPEED

---

## 1. 概要

Guided モードで矩形（四角形）の頂点を順に飛行する Python スクリプトの設計計画。
JSON パラメータから設定を読み込み、離陸 → 矩形飛行 → 着陸のシーケンスを自動実行する。
円飛行 (`circle_flight.py`) のアーキテクチャをベースに、四角形のエッジ移動と頂点停止のロジックを追加する。

### ユーザー要件

| # | 要件 | 詳細 |
|---|------|------|
| 1 | 四角形の頂点を順に移動 | 4頂点 (V0→V1→V2→V3→V0→...) |
| 2 | 各頂点で指定秒数停止 | ホバリング（位置維持） |
| 3 | 一辺を移動する速度を指定 | エッジ移動時の対地速度 [m/s] |
| 4 | JSON パラメータで設定 | 既存 `circle_params.json` と同パターン |

---

## 2. 技術調査

### 2.1 調査対象

| # | 調査項目 | 情報源 |
|---|----------|--------|
| 1 | GUIDED モードでの速度制御の可否 | [ArduPilot Copter Commands in Guided Mode](https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html) |
| 2 | SET_POSITION_TARGET_GLOBAL_INT の type_mask 仕様 | [MAVLink Common Message Set](https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT) + ArduPilot 実装 |
| 3 | MAV_CMD_DO_CHANGE_SPEED の有効性 | [ArduPilot MAV_CMD リファレンス](https://ardupilot.org/copter/docs/common-mavlink-mission-command-messages-mav_cmd.html) |
| 4 | DroneKit / 他実装の速度制御パターン | [DroneKit Python Guide](https://dronekit-python.readthedocs.io/en/latest/guide/copter/guided_mode.html) |

### 2.2 SET_POSITION_TARGET_GLOBAL_INT の type_mask 解析

#### 2.2.1 ArduPilot の type_mask 定義（1-indexed, pymavlink と互換）

```
bit1: PosX  (緯度)       │ bit4: VelX  (vx, 北方向速度)
bit2: PosY  (経度)       │ bit5: VelY  (vy, 東方向速度)
bit3: PosZ  (高度)       │ bit6: VelZ  (vz, 下方向速度)
                         │ bit7: AccX  (afx)
                         │ bit8: AccY  (afy)
                         │ bit9: AccZ  (afz)
                         │ bit11: yaw
                         │ bit12: yaw_rate
```

> **注意**: ArduPilot のドキュメントでは bit が 1-indexed で表記されている。
> pymavlink の MAVLink 標準では POSITION_TARGET_TYPEMASK が 0-indexed で、
> `0x0001: VX_IGNORE, 0x0002: VY_IGNORE, ...` のように割り当てが異なる。
> 本計画では ArduPilot の 1-indexed 表記に従う。

#### 2.2.2 主要マスク値一覧（SET_POSITION_TARGET_GLOBAL_INT）

| 用途 | マスク (hex) | マスク (dec) | 制御対象 | 無視対象 |
|------|-------------|-------------|----------|----------|
| **位置のみ** | `0x0DF8` | 3576 | PosX, PosY, PosZ | Vel, Acc, yaw, yaw_rate |
| **速度のみ** | `0x0DC7` | 3527 | VelX, VelY, VelZ | Pos, Acc, yaw_rate |
| **位置+速度** | `0x0DC0` | 3520 | Pos, Vel 全軸 | Acc, yaw_rate |
| **位置+速度+加速度** | `0x0C00` | 3072 | Pos, Vel, Acc 全軸 | yaw_rate |
| **yaw のみ** | `0x09FF` | 2559 | yaw | Pos, Vel, Acc, yaw_rate |
| **yaw_rate のみ** | `0x05FF` | 1535 | yaw_rate | Pos, Vel, Acc, yaw |

#### 2.2.3 circle_flight.py で使用されているマスク

```python
mask = "0x09F8"  # = 0b0000 1001 1111 1000
```

- 制御対象: PosX(lat), PosY(lon), VelX, VelY, VelZ
- 無視対象: PosZ(alt), Acc 全軸, yaw, yaw_rate
- **特徴**: 高度 (PosZ) を無視しているが、実運用上は高度値も送信しており、
  ArduPilot の位置コントローラが高度目標として解釈している
### 2.3 GUIDED モードでの速度制御：方式比較

#### 2.3.1 方式A: 位置制御 + 細かい waypoint 分割（推奨）

**仕組み**: エッジを細かい区間に分割し、各点を `send_rate_hz` で順次送信

```
[エッジ長 L, 速度 v, 送信レート f]
  → 移動時間 T = L/v
  → 分割数 N = ceil(T * f)
  → 各点間隔 d = L/N
  → dt = T/N = 1/f 秒間隔で連続送信
```

**利点**:
- 円飛行 (`circle_flight.py`) で実績あり。Pixhawk6C で安定動作確認済み
- 位置フィードバックがかかるためドリフトしない
- ArduPilot の位置コントローラが滑らかな軌道補間を行う
- エッジ上での速度管理が間接的に可能（dt 間隔と移動距離から速度が決まる）

**欠点**:
- 計算負荷がやや高い（リアルタイムで GPS 座標を計算）
- 頂点到達判定が必要（位置誤差の閾値管理）

#### 2.3.2 方式B: SET_POSITION_TARGET_GLOBAL_INT の速度フィールドを使用

**仕組み**: type_mask = `0x0DC7`（速度制御）で vx, vy を設定

```python
# 例: 北方向に 1m/s で移動
master.mav.set_position_target_global_int_send(
    0, target_sys, target_comp,
    MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
    0x0DC7,  # 速度制御マスク
    0, 0, 0,           # 位置（無視）
    vx, vy, vz,        # 速度 [m/s]
    0, 0, 0,           # 加速度（無視）
    0, 0,              # yaw, yaw_rate（無視）
)
```

**利点**:
- エッジ移動のロジックが単純（速度ベクトルを設定するだけ）
- 速度指定が直接的

**欠点**:
- **3秒以内に再送しないと停止する**（ArduPilot 仕様）→ 送信忘れのリスク
- オープンループ制御のため、風などの外乱で位置がずれる可能性
- 位置フィードバックがないため、頂点到達の正確な判定が困難
- Pixhawk6C 実機での動作確認実績なし
#### 2.3.3 方式C: MAV_CMD_DO_CHANGE_SPEED + 位置制御の組み合わせ

**仕組み**:
1. `MAV_CMD_DO_CHANGE_SPEED` で目標速度を設定（`WPNAV_SPEED` 相当のパラメータ変更）
2. `SET_POSITION_TARGET_GLOBAL_INT` (位置制御) で目標位置を送信
3. ArduPilot の位置コントローラが `WPNAV_SPEED` 制限内で移動

**利点**:
- 位置制御の安定性と速度制御を両立できる可能性

**欠点**:
- `MAV_CMD_DO_CHANGE_SPEED` が GUIDED モードの位置制御にどの程度影響するか不透明
  （主に AUTO ミッション向けのコマンド）
- パラメータ変更による副作用の可能性（`WPNAV_SPEED` が他の動作にも影響）
- 実機検証が必要でリスクが高い

#### 2.3.4 方式D: ハイブリッド方式（方式A + 方式B）

**仕組み**:
- エッジ移動中: 方式B（速度制御）で効率的に移動
- 頂点接近時: 方式A（位置制御）に切り替えて正確に頂点に停止
- 頂点停止中: 位置制御でホバリング維持

**利点**: 両方式の長所を活かせる
**欠点**: 実装が複雑（マスクの動的切り替え）、方式B の実機検証が必要

### 2.4 推奨方式の選定

**→ 方式A（位置制御 + 細かい waypoint 分割）を採用する。**

**選定理由**:
1. `circle_flight.py` で Pixhawk6C 実機での安定動作が確認済み
2. 既存のコードベース（`_send_setpoint()`, `gps_to_local_xyz()`, `local_xyz_to_gps()`）を
   ほぼそのまま再利用可能
3. 位置フィードバックによるロバスト性（風などの外乱に強い）
4. 頂点での正確な位置停止が容易（位置制御マスクで同じ頂点を連続送信するだけ）
5. 速度制御は `send_rate_hz` × 分割間隔 で間接的に実現（実績あり）
---

## 3. 矩形飛行アルゴリズム設計

### 3.1 数学的定式化

#### 3.1.1 矩形の頂点定義

中心座標 (lat_c, lon_c)、一辺の長さ S [m] とする。
矩形の 4 頂点をローカル座標系（東=x, 北=y）で以下のように定義する：

```
V0 = ( x_c - S/2,  y_c - S/2 )    # 南西
V1 = ( x_c + S/2,  y_c - S/2 )    # 南東
V2 = ( x_c + S/2,  y_c + S/2 )    # 北東
V3 = ( x_c - S/2,  y_c + S/2 )    # 北西
```

> 中心座標 (lat_c, lon_c) はローカル座標の原点 (x_c=0, y_c=0) に対応する。

エッジの接続順序（反時計回り）:
```
V0 → V1 → V2 → V3 → V0 → V1 → ...
```

各エッジの方向ベクトル:
```
E0 (V0→V1): (dx, dy) = (+S,  0)   # 東方向
E1 (V1→V2): (dx, dy) = ( 0, +S)   # 北方向
E2 (V2→V3): (dx, dy) = (-S,  0)   # 西方向
E3 (V3→V0): (dx, dy) = ( 0, -S)   # 南方向
```

#### 3.1.2 エッジ上の waypoint 生成

エッジ長 S [m]、目標速度 v [m/s]、送信レート f [Hz] とする。

```
移動時間:    T = S / v            [秒]
分割数:      N = ceil(T * f)      [点]
各点間隔:    d = S / N            [m]
時間間隔:    dt = T / N = 1/f     [秒]
```

エッジ E_k 上の i 番目の waypoint (i = 0, 1, ..., N):

```
比率:       r = i / N           (0 → 1)
位置:       (x_i, y_i) = V_k + r * (V_{k+1} - V_k)
```

エッジの終点 (i=N) は次の頂点 V_{k+1} と一致する。

#### 3.1.3 タイミング制御

```
エッジ移動中:
  t_elapsed = 0 → T [秒]
  waypoint を dt = 1/f 間隔で送信
  送信時点の x, y を線形補間で計算

頂点停止中:
  t_stop = stop_time_sec [秒]
  頂点の座標を dt 間隔で再送信（位置維持）
  ArduPilot が位置ホバリング制御を行う
```

#### 3.1.4 GPS 座標変換（既存関数を再利用）

```python
# ローカル座標 → GPS座標（circle_flight.py の local_xyz_to_gps と同一）
lat = ref_lat + y / 111319.5
lon = ref_lon + x / (111319.5 * cos(ref_lat * π / 180))
### 3.2 状態遷移設計

```
[INIT]
  │
  ▼
[WAITING_ARM]  ←─ Guided+Arm 待機
  │
  ▼
[TAKEOFF]  ←─ 離陸指令 → 高度到達待機
  │
  ▼
[SQUARE_START]  ←─ 最初の頂点 V0 へ移動
  │
  ▼
[SQUARE_EDGE]  ←─ エッジ上の waypoint を連続送信
  │
  ▼ (エッジ終点到達)
[SQUARE_VERTEX_STOP]  ←─ 頂点で stop_time_sec 秒ホバリング
  │
  ├── 次のエッジへ (SQUARE_EDGE に戻る)
  │
  ▼ (全エッジ完了)
[SQUARE_COMPLETE]  ←─ 離陸地点上空へ帰還
  │
  ▼
[LANDING]  ←─ LAND モード切替 → 着陸検出
  │
  ▼
[COMPLETE]
```

| 状態 | 内容 | 遷移条件 |
|------|------|----------|
| `WAITING_ARM` | Guided+Arm 待機 | Arm 検出 → TAKEOFF |
| `TAKEOFF` | 離陸 + 高度到達待機 | 高度 >= target*0.9 → SQUARE_START |
| `SQUARE_START` | 頂点 V0 へ移動 | 距離 < threshold → SQUARE_EDGE |
| `SQUARE_EDGE` | エッジ上の waypoint 連続送信 | エッジ終点到達 → SQUARE_VERTEX_STOP |
| `SQUARE_VERTEX_STOP` | 頂点で stop_time_sec 秒停止 | 停止時間経過 → SQUARE_EDGE or SQUARE_COMPLETE |
| `SQUARE_COMPLETE` | 離陸地点上空へ帰還 | 距離 < threshold → LANDING |
| `LANDING` | 着陸 | ディスアーム or 高度安定 → COMPLETE |
| `COMPLETE` | 終了 | — |

### 3.3 頂点到達判定

エッジ移動中、現在位置とエッジ終点（次の頂点）の距離を監視する：

```python
dist_to_target = sqrt((x_now - x_target)^2 + (y_now - y_target)^2)
```

閾値:
- 基本: `side_length_m * 0.1`（一辺の長さの 10%）
- 最小: `0.3m`（GPS 精度を考慮した下限）
- 全 waypoint 送信完了時点で強制的に頂点到達とみなす

### 3.4 安全停止判定（circle_flight.py の仕組みを継承）

- ディスアーム検出 → 即時 `_running = False`
- モード変更検出（Guided → 他モード）→ 安全停止
---

## 4. JSON パラメータスキーマ設計

### 4.1 設定ファイル仕様

ファイル名: `square_params.json`（デフォルト）

```jsonc
{
  // ===== 矩形飛行設定 =====
  "center": {
    "latitude": 36.0757722,        // 矩形の中心緯度 [deg]
    "longitude": 136.2132926       // 矩形の中心経度 [deg]
  },
  "side_length_m": 5.0,            // 一辺の長さ [m]（> 0）

  "altitude_m": 1.2,               // 飛行高度 [m]（>= 0.5）

  // ===== 速度設定 =====
  "edge_speed_m_s": 1.0,           // エッジ移動速度 [m/s]
                                   // （> 0, 推奨 0.3〜3.0）

  // ===== 停止設定 =====
  "stop_time_sec": 2.0,            // 各頂点での停止時間 [秒]（>= 0）

  // ===== 周回設定 =====
  "num_laps": 3,                   // 周回数（>= 1）
                                   // 1周 = V0→V1→V2→V3→V0

  // ===== 方向設定 =====
  "direction": "CCW",              // "CW"（時計回り: V0→V3→V2→V1）
                                   // "CCW"（反時計回り: V0→V1→V2→V3）

  // ===== ヨー角設定 =====
  "yaw_mode": "fixed",             // "fixed" / "edge" / "center"
  "fixed_yaw_deg": 180.0,          // yaw_mode="fixed" 時のヨー角 [deg]

  // ===== 離陸設定 =====
  "takeoff_alt_m": 0.3,

  // ===== 通信設定 =====
  "send_rate_hz": 10,              // 送信レート [Hz]
  "mask": "0x0DF8",                // 位置制御マスク

  // ===== 安全設定 =====
  "land_after": true,
  "loiter_after_takeoff_sec": 3.0,
  "loiter_before_land_sec": 3.0,
  "arrival_threshold_m": null      // 頂点到達閾値 [m]（null=side_length*0.1）
}
```

### 4.2 バリデーションルール

| パラメータ | 制約 | デフォルト値 |
|------------|------|-------------|
| `center.latitude` | -90.0 〜 90.0 | (必須) |
| `center.longitude` | -180.0 〜 180.0 | (必須) |
| `side_length_m` | > 0.0 | 5.0 |
| `altitude_m` | >= 0.5 | 2.0 |
| `edge_speed_m_s` | > 0.0, <= 5.0（超は警告） | 1.0 |
| `stop_time_sec` | >= 0.0 | 2.0 |
| `num_laps` | >= 1 | 1 |
| `direction` | "CW" or "CCW" | "CCW" |
| `yaw_mode` | "fixed" / "edge" / "center" | "fixed" |
| `fixed_yaw_deg` | 0.0 〜 360.0 | 0.0 |
| `send_rate_hz` | >= 1 | 10 |
| `arrival_threshold_m` | > 0.0（指定時） | side_length_m * 0.1 |

### 4.3 自動計算パラメータ

| パラメータ | 計算式 | 説明 |
|------------|--------|------|
| エッジ移動時間 | `T_edge = side_length_m / edge_speed_m_s` | 1エッジの移動時間 [秒] |
| エッジ分割数 | `N_edge = ceil(T_edge * send_rate_hz)` | エッジ上の waypoint 数 |
| 1周時間 | `T_lap = 4*(T_edge + stop_time_sec)` | 4エッジ + 4頂点停止 |
---

## 5. 全体アーキテクチャ設計

### 5.1 クラス構成

```
square_flight.py
├── SquareFlightController （メインコントローラークラス）
│   ├── __init__(self, param_path: str)
│   ├── connect(self) -> bool
│   ├── wait_for_guided_arm(self) -> bool
│   ├── takeoff(self) -> bool
│   ├── move_to_start_vertex(self) -> bool       ← 頂点 V0 へ移動
│   ├── execute_edge(self, v_start, v_end) -> bool  ← 1エッジの飛行
│   ├── stop_at_vertex(self, vertex, duration) -> bool ← 頂点停止
│   ├── execute_square_flight(self) -> bool      ← 矩形飛行のメインループ
│   ├── return_to_takeoff(self) -> bool
│   ├── land(self) -> bool
│   ├── run(self) -> None
│   └── cleanup(self) -> None
│
├── ヘルパー関数（モジュールレベル）
│   ├── gps_to_local_xyz(lat, lon, alt, ref_lat, ref_lon, ref_alt) -> (x, y, z)
│   ├── local_xyz_to_gps(x, y, z, ref_lat, ref_lon, ref_alt) -> (lat, lon, alt)
│   ├── generate_square_vertices(center, side_length, direction) -> list[(x,y)]
│   ├── generate_edge_waypoints(v_start, v_end, n) -> list[(x,y)]
│   ├── load_params(path) -> dict
│   └── _validate_params(params) -> None
```

### 5.2 スレッド構成（circle_flight.py と同一）

| スレッド | 役割 | 起動タイミング | 停止条件 |
|----------|------|---------------|----------|
| **メイン** | 飛行シーケンス制御 | スクリプト開始 | シーケンス完了 |
| **モニター** | HEARTBEAT/GPS 監視、ディスアーム検出 | `connect()` 後 | `self._running=False` |
| **レコード** | CSV データ記録 | `connect()` 後 | `self._running=False` |

### 5.3 データフロー

```
[JSON パラメータ] → load_params() → [パラメータ検証]
    ↓
[矩形頂点生成] generate_square_vertices() → 4頂点 (x,y)
    ↓
[エッジwaypoint生成] generate_edge_waypoints() × 4エッジ
    ↓
[MAVLink接続] connect() → [モニター/レコード スレッド起動]
    ↓
[飛行シーケンス]
    ├── wait_for_guided_arm()
    ├── takeoff()
    ├── move_to_start_vertex()
    ├── execute_square_flight()
    │   └── for lap in range(num_laps):
    │       └── for edge in range(4):
    │           ├── execute_edge()
    │           └── stop_at_vertex()
    ├── return_to_takeoff()
    └── land()
    ↓
[CSV保存] save_csv() → ~/LOGS_Pixhawk6c/
    ↓
[クリーンアップ] cleanup()
```

### 5.4 CSV 記録仕様（circle_flight.py と同一フォーマット）

```csv
Time, GPS_X, GPS_Y, GPS_Z, Target_X, Target_Y, Target_Z
```

- 保存先: `~/LOGS_Pixhawk6c/`
---

## 6. 詳細関数設計

### 6.1 `generate_square_vertices(center, side_length, direction)`

4 頂点のローカル座標 (x, y) を生成し、direction に応じた訪問順に並べる。

```python
def generate_square_vertices(center, side_length, direction):
    """
    Args:
        center: {'latitude': float, 'longitude': float}
        side_length: 一辺の長さ [m]
        direction: "CW" or "CCW"

    Returns:
        [(x, y), ...] — 4頂点のローカル座標（訪問順）
    """
    half = side_length / 2.0

    # 4頂点のローカル座標（東=x, 北=y）
    V0 = (-half, -half)   # 南西
    V1 = (+half, -half)   # 南東
    V2 = (+half, +half)   # 北東
    V3 = (-half, +half)   # 北西

    if direction == "CCW":
        return [V0, V1, V2, V3]   # 反時計回り
    else:  # CW
        return [V0, V3, V2, V1]   # 時計回り
```

### 6.2 `generate_edge_waypoints(v_start, v_end, n)`

エッジ上の n 個の waypoint を線形補間で生成する。

```python
def generate_edge_waypoints(v_start, v_end, n):
    """
    Args:
        v_start: 始点のローカル座標 (x, y)
        v_end: 終点のローカル座標 (x, y)
        n: 分割数（waypoint 数、始点含む）

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
```

### 6.3 `execute_edge(self, v_start, v_end)`

1 エッジの飛行を実行する。

```python
def execute_edge(self, v_start, v_end):
    """
    エッジ V_start → V_end を飛行する。

    1. エッジの waypoint リストを生成
    2. send_rate_hz のレートで連続送信
    3. 全 waypoint 送信完了で終了

    Returns:
        bool: 成功時 True
    """
    dx = v_end[0] - v_start[0]
    dy = v_end[1] - v_start[1]
    edge_length = math.sqrt(dx**2 + dy**2)
    edge_time = edge_length / self.params["edge_speed_m_s"]

    send_hz = self.params["send_rate_hz"]
    n_points = max(2, int(edge_time * send_hz))
    dt = 1.0 / send_hz

    wps = generate_edge_waypoints(v_start, v_end, n_points)

    edge_start = time.time()
    for i, (x, y) in enumerate(wps):
        if not self._running:
            return False

        lat, lon, _ = local_xyz_to_gps(
            x, y, self.params["altitude_m"],
            self._ref_lat, self._ref_lon, self._ref_alt
        )
        yaw = self._get_yaw(v_start, v_end)
        self._send_setpoint(lat, lon,
                            self.params["altitude_m"], yaw)

        # 次の送信タイミングまで待機
        next_time = edge_start + (i + 1) * dt
        sleep_time = next_time - time.time()
        if sleep_time > 0:
            time.sleep(sleep_time)

    return True
```

### 6.4 `stop_at_vertex(self, vertex, duration)`

頂点で指定時間停止（位置を再送信し続ける）。

```python
def stop_at_vertex(self, vertex, duration):
    """
    頂点で duration 秒間ホバリングする。
    vertex の座標を send_rate_hz で繰り返し送信。

    Args:
        vertex: 頂点のローカル座標 (x, y)
        duration: 停止時間 [秒]

    Returns:
        bool: 成功時 True
    """
    if duration <= 0:
        return True

    x, y = vertex
    lat, lon, _ = local_xyz_to_gps(
        x, y, self.params["altitude_m"],
        self._ref_lat, self._ref_lon, self._ref_alt
    )

    send_hz = self.params["send_rate_hz"]
    dt = 1.0 / send_hz
    yaw = self.params["fixed_yaw_deg"]

    stop_start = time.time()
    print(f"  頂点停止 {duration}秒...")

    while time.time() - stop_start < duration:
        if not self._running:
            return False
        self._send_setpoint(lat, lon,
                            self.params["altitude_m"], yaw)
        time.sleep(dt)

    print(f"  ✓ 頂点停止完了")
    return True
```

### 6.5 `execute_square_flight(self)`

矩形飛行のメインループ。

```python
def execute_square_flight(self):
    """
    矩形飛行を実行する。

    for lap in range(num_laps):
        エッジ0 → 頂点停止 → エッジ1 → 頂点停止 → ...
         → エッジ3 → 最終周回以外、頂点停止

    最終エッジの終点は V0（閉じた矩形のため）。
    """
    vertices = generate_square_vertices(
        self.params["center"],
        self.params["side_length_m"],
        self.params["direction"]
    )
    stop_time = self.params["stop_time_sec"]
    num_laps = self.params["num_laps"]

    self._state = self.STATE_SQUARE_FLYING

    for lap in range(num_laps):
        if not self._running:
            return False
        print(f"\n--- 周回 {lap + 1}/{num_laps} ---")

        for i in range(4):
            v_start = vertices[i]
            v_end = vertices[(i + 1) % 4]

            print(f"  エッジ {i}: V{i} → V{(i+1)%4}")
            if not self.execute_edge(v_start, v_end):
                return False

            # 最終エッジかつ最終周回では停止しない
            is_last = (lap == num_laps - 1) and (i == 3)
            if not is_last:
                if not self.stop_at_vertex(v_end, stop_time):
                    return False
                print(f"  ✓ 頂点 V{(i+1)%4} 到達")

---

## 7. 方式B（速度制御）の補足検討

将来的な拡張オプションとして、方式B（SET_POSITION_TARGET_GLOBAL_INT の速度フィールド使用）
についても実装方針を記録しておく。

### 7.1 速度制御マスクの指定

```python
# 速度制御用マスク
VELOCITY_MASK = 0x0DC7  # 3527 decimal

def _send_velocity(self, vx, vy, vz=0, yaw_deg=None):
    """
    速度制御コマンドを送信する。
    vx: 北方向速度 [m/s]
    vy: 東方向速度 [m/s]
    """
    yaw = math.radians(yaw_deg) if yaw_deg is not None else 0

    self.master.mav.set_position_target_global_int_send(
        0,
        self.master.target_system,
        self.master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        VELOCITY_MASK,
        0, 0, 0,                # 位置（無視）
        vx, vy, vz,             # 速度
        0, 0, 0,                # 加速度（無視）
        yaw, 0,                 # yaw, yaw_rate
    )
```

### 7.2 速度制御の注意点

1. **再送要件**: 最低 1 秒に 1 回の再送が必要（3 秒でタイムアウト）
2. **頂点停止**: 頂点に近づいたら速度マスク → 位置マスク（`0x0DF8`）に切り替える
3. **位置フィードバック**: GPS による現在位置監視が必須
4. **風の影響**: 速度制御は対気速度ではなく対地速度。強風時は目標速度を維持できない可能性

### 7.3 ハイブリッド方式の推奨実装パターン（将来検討用）

```
エッジ開始:
  1. 速度制御マスク(0x0DC7) + エッジ方向の速度ベクトル で移動開始
  2. 1秒間隔で速度コマンドを再送

頂点接近時（残り距離 < side_length * 0.2）:
  3. 位置制御マスク(0x0DF8) + 頂点座標 に切り替え
  4. 頂点到達を位置フィードバックで判定

頂点停止:
  5. 位置制御マスクで頂点座標を連続送信（ホバリング）
```

---

## 8. 実装上の注意点

### 8.1 circle_flight.py からの継承要素

| 要素 | 再利用方法 |
|------|-----------|
| `gps_to_local_xyz()` | そのままコピー |
| `local_xyz_to_gps()` | そのままコピー |
| `_send_setpoint()` | マスク値を `0x0DF8` に変更 |
| `connect()` | ほぼそのまま継承 |
| `wait_for_guided_arm()` | そのまま継承 |
| `takeoff()` | そのまま継承 |
| `land()` | そのまま継承 |
| `return_to_takeoff()` | そのまま継承 |
| `_monitor_loop()` / `_record_loop()` | そのまま継承 |
| `cleanup()` / `_print_params()` | そのまま継承 |
| `load_params()` | パラメータ項目を差し替え |
| `_validate_params()` | 拡張して square 用制約を追加 |

### 8.2 新規実装が必要な要素

| 要素 | 説明 |
|------|------|
| `generate_square_vertices()` | 矩形の 4 頂点をローカル座標で生成 |
| `generate_edge_waypoints()` | エッジ上の waypoint を線形補間で生成 |
| `execute_edge()` | 1 エッジの飛行 |
| `stop_at_vertex()` | 頂点でのホバリング停止 |
| `execute_square_flight()` | 矩形飛行のメインループ |
| `move_to_start_vertex()` | 最初の頂点 V0 へ移動 |
| `_get_yaw()` | yaw_mode に応じたヨー角計算 |
| `square_params.json` | 新しい JSON パラメータファイル |

### 8.3 マスク値の変更

| 項目 | circle_flight.py | square_flight.py | 理由 |
|------|-----------------|-----------------|------|
| mask | `0x09F8` | `0x0DF8` | ArduPilot 推奨の「位置のみ制御」マスク。<br>`0x0DF8` = PosX/Y/Z 制御, Vel/Acc/yaw_rate 無視。<br>より明示的な位置制御で、高度も制御対象に含める。 |

> **参考**: ArduPilot 公式ドキュメントの位置制御マスク:
> - `0x0DF8` (3576): PosX/Y/Z 制御、VelX/Y/Z, AccX/Y/Z, yaw, yaw_rate 無視
> - `0x09F8` (circle_flight.py): PosX/Y 制御、**PosZ 無視**、VelX/Y/Z 制御、Acc 無視
> 
> square_flight では一定高度を維持するため、高度制御が明示的に有効な `0x0DF8` を推奨。

---

## 9. ヨー角制御の設計

### 9.1 モード一覧

| モード | 動作 | 計算 |
|--------|------|------|
| `fixed` | 指定された固定ヨー角を維持 | `yaw = fixed_yaw_deg` |
| `edge` | エッジの進行方向を向く | `yaw = atan2(dx, dy)` [deg] |
| `center` | 矩形の中心を向く | 現在位置から中心への方位角 |

### 9.2 `edge` モードの実装

```python
def _calc_edge_yaw(v_start, v_end):
    """エッジの進行方向のヨー角 [deg] を計算する。"""
    dx = v_end[0] - v_start[0]  # 東方向
    dy = v_end[1] - v_start[1]  # 北方向
    # atan2(dx, dy): dx=East, dy=North, 北=0°
    return math.degrees(math.atan2(dx, dy)) % 360
```

---

## 10. ファイル構成

```
/home/taki/Mavlink_raspi/EKF/
├── square_flight.py          # メインスクリプト（新規作成）
├── square_flight_plan.md     # 本計画ドキュメント
├── square_params.json        # デフォルトパラメータ（新規作成）
├── circle_flight.py          # 参照元（既存）
├── circle_flight_plan.md     # 参照元（既存）
└── circle_params.json        # 参照元（既存）
```

---

## 11. 参考リンク

| 項目 | URL |
|------|-----|
| ArduPilot Copter Commands in Guided Mode | https://ardupilot.org/dev/docs/copter-commands-in-guided-mode.html |
| MAVLink SET_POSITION_TARGET_GLOBAL_INT | https://mavlink.io/en/messages/common.html#SET_POSITION_TARGET_GLOBAL_INT |
| ArduPilot MAV_CMD_DO_CHANGE_SPEED | https://ardupilot.org/copter/docs/common-mavlink-mission-command-messages-mav_cmd.html#mav-cmd-do-change-speed |
| DroneKit Guided Mode | https://dronekit-python.readthedocs.io/en/latest/guide/copter/guided_mode.html |
| ArduPilot GCS_Mavlink.cpp (Copter) | https://github.com/ArduPilot/ardupilot/blob/master/ArduCopter/GCS_Mavlink.cpp |
