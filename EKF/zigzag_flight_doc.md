# zigzag_flight.py — ジグザグ（蛇行）飛行ドキュメント

## 概要

`zigzag_flight.py` は ArduPilot の Guided モードを使用して、矩形エリア内を
ジグザグ（蛇行）飛行させるスクリプトです。

矩形エリア内を水平（または垂直）のセグメントで折り返しながら、
垂直（または水平）方向に進行するパターンを飛行します。

測量・空撮・探索などのアプリケーションで、エリアを効率的にカバーするために使用します。

## 実行方法

```bash
# デフォルトのパラメータファイルを使用
python zigzag_flight.py

# カスタムパラメータファイルを指定
python zigzag_flight.py /path/to/custom_params.json
```

### 事前準備

1. `setup_EKF_Observer4.py` を実行して ArduPilot のパラメータを設定してください。
   このスクリプトは ArduPilot が以下の状態であることを前提とします：
   - Guided モード
   - Arm 済み
   - GPS ロック取得済み

2. Pixhawk6C が `/dev/ttyAMA0` にシリアル接続されていることを確認してください。

### 実行シーケンス

1. MAVLink 接続確立（`/dev/ttyAMA0`, 1000000bps）
2. Guided モード + Arm 検出待機
3. 離陸（`takeoff_alt_m` まで上昇）
4. ジグザグ開始位置（`start_corner` で指定されたコーナー）へ移動
5. ジグザグ飛行実行
6. 離陸地点上空へ帰還
7. 着陸

## パラメータ一覧

| パラメータ | 型 | デフォルト | 説明 |
|---|---|---|---|
| `center.latitude` | float | 0.0 | 矩形エリアの中心緯度 [deg]（-90.0 〜 90.0） |
| `center.longitude` | float | 0.0 | 矩形エリアの中心経度 [deg]（-180.0 〜 180.0） |
| `half_width_m` | float | 5.0 | 矩形の横幅の半分 [m]（> 0） |
| `half_height_m` | float | 5.0 | 矩形の縦幅の半分 [m]（> 0） |
| `speed_m_s` | float | 1.0 | 飛行速度 [m/s]（> 0） |
| `altitude_m` | float | 0.6 | 飛行高度 [m]（>= 0.5） |
| `stop_at_turn_sec` | float | 2.0 | 折返し点での停止時間 [秒]（>= 0） |
| `num_zigs` | int | 3 | 折り返し回数 = 水平/垂直セグメントの本数（>= 2） |
| `zigzag_axis` | string | "horizontal" | ジグザグの軸："horizontal"（東西セグメント＋南北進行） または "vertical"（南北セグメント＋東西進行） |
| `start_corner` | string | "NE" | 開始コーナー："NE", "NW", "SE", "SW" のいずれか |
| `takeoff_alt_m` | float | 0.5 | 離陸高度 [m] |
| `send_rate_hz` | int | 10 | セットポイント送信レート [Hz]（>= 1） |
| `yaw_mode` | string | "fixed" | ヨー制御モード："fixed"（固定）/ "edge"（進行方向）/ "center"（中心向き） |
| `fixed_yaw_deg` | float | 0.0 | yaw_mode="fixed" 時の固定ヨー角 [deg]（0.0 〜 360.0, 北=0°, 東=90°） |
| `land_after` | bool | true | 飛行完了後に着陸するか |
| `loiter_after_takeoff_sec` | float | 3.0 | 離陸後の安定待機時間 [秒] |
| `loiter_before_land_sec` | float | 3.0 | 着陸前の安定待機時間 [秒] |
| `loiter_at_start_sec` | float | 3.0 | ジグザグ開始位置到着後のホバリング待機時間 [秒]（>= 0） |
| `mask` | string | "0x09F8" | セットポイントの制御マスク（16進数文字列） |

### パラメータのバリデーション

- `num_zigs >= 2`：2未満の場合はエラー
- `zigzag_axis`："horizontal" または "vertical" のみ
- `start_corner`："NE", "NW", "SE", "SW" のいずれかのみ
- その他のパラメータは `square_flight.py` と同様のバリデーション

## 飛行パターンの図解

### zigzag_axis="horizontal", start_corner="NE", num_zigs=3

```
  北(y+)
  ▲
  │  NW(-w,+h) ←──── セグメント1 ──── NE(+w,+h) [開始]
  │    │                                │
  │    │ コネクタ1                       │
  │    │ (南へstep)                      │
  │    ▼                                ▼
  │  (-w, y1) ──── セグメント2 ────► (+w, y1)
  │    │                                │
  │    │ コネクタ2                       │
  │    │ (南へstep)                      │
  │    ▼                                ▼
  │  SW(-w,-h) ←──── セグメント3 ──── SE(+w,-h) [終了]
  │
  └──────────────────────────────────────► 東(x+)
```

### zigzag_axis="vertical", start_corner="NE", num_zigs=3

```
  北(y+)
  ▲
  │  NW(-w,+h)           NE(+w,+h) [開始]
  │                          │
  │                          │ セグメント1 (南へ)
  │                          │
  │                          ▼
  │  (-w, -h)  ◄── コネクタ1 ──  (+w,-h) 
  │     │
  │     │ セグメント2 (北へ)
  │     │
  │     ▼
  │  (-w,+h)  ── コネクタ2 ──► (+w,+h)
  │                                │
  │                                │ セグメント3 (南へ)
  │                                │
  │                                ▼
  │  SW(-w,-h)                  SE(+w,-h) [終了]
  │
  └──────────────────────────────────────► 東(x+)
```

### 頂点生成ロジック

1. `step = 2 * (zigzag_axisがhorizontalならhalf_height, verticalならhalf_width) / (num_zigs - 1)`
2. `num_zigs` 個の行（horizontal）/ 列（vertical）を生成
3. 各行/列で：
   - セグメントの始点と終点を計算
   - 偶数セグメントと奇数セグメントで進行方向が反転
4. セグメント間はコネクタ（`step` 分だけ垂直/水平移動）

開始コーナーからの初期進行方向：
- `zigzag_axis="horizontal"` の場合: 西端のコーナー(NW/SW)からは東へ、東端のコーナー(NE/SE)からは西へ
- `zigzag_axis="vertical"` の場合: 北端のコーナー(NE/NW)からは南へ、南端のコーナー(SE/SW)からは北へ

## CSVデータ

飛行データは `~/LOGS_Pixhawk6c/{YYYYMMDD_HHMMSS}_zigzag.csv` に保存されます。

CSVフォーマット:
```
Time, GPS_X, GPS_Y, GPS_Z, Target_X, Target_Y, Target_Z
```

- GPS座標は矩形中心を原点とするローカル直交座標系（東=x, 北=y, 上=z）[m]
- Targetは送信した目標位置（同座標系）

## 使用例

### 例1: 基本的なジグザグ飛行（東西方向、NEスタート）
```json
{
  "center": { "latitude": 36.0757754, "longitude": 136.2132908 },
  "half_width_m": 2.0,
  "half_height_m": 3.0,
  "num_zigs": 5,
  "zigzag_axis": "horizontal",
  "start_corner": "NE",
  "speed_m_s": 1.0,
  "altitude_m": 2.0,
  "stop_at_turn_sec": 1.0,
  "yaw_mode": "edge"
}
```
→ 4m×6m のエリアを東西5セグメントでカバー。ヨーは進行方向を向く。

### 例2: 垂直ジグザグ、SWスタート
```json
{
  "center": { "latitude": 36.0757754, "longitude": 136.2132908 },
  "half_width_m": 3.0,
  "half_height_m": 2.0,
  "num_zigs": 4,
  "zigzag_axis": "vertical",
  "start_corner": "SW",
  "speed_m_s": 1.5,
  "altitude_m": 1.5,
  "stop_at_turn_sec": 0.5,
  "yaw_mode": "center"
}
```
→ 6m×4m のエリアを南北4セグメントでカバー。ヨーは常に中心を向く。

## 依存関係

- Python 3.6+
- `pymavlink` — MAVLink 通信
- `pytz` — タイムゾーン処理（日本時間 JST）

インストール:
```bash
pip install pymavlink pytz
```

## setup_EKF_Observer4.py との連携

このスクリプトを実行する前に、`setup_EKF_Observer4.py` で ArduPilot の
パラメータ設定を完了しておく必要があります。

`setup_EKF_Observer4.py` は以下を設定します：
- EKF オブザーバー関連パラメータ
- シリアルポート設定
- GPS 設定
- その他飛行に必要な ArduPilot パラメータ

設定後、手動または自動で Guided モード + Arm の状態にしてください。

## ファイル構成

```
EKF/
├── zigzag_flight.py       — メインプログラム
├── zigzag_params.json     — デフォルトパラメータファイル
├── zigzag_flight_doc.md   — 本ドキュメント
├── setup_EKF_Observer4.py — ArduPilot パラメータ設定（事前実行必須）
├── square_flight.py       — 四角形飛行（姉妹プログラム）
└── square_params.json     — 四角形飛行パラメータ
```

## 安全上の注意

- 飛行前に必ず `setup_EKF_Observer4.py` でパラメータを設定してください
- 初回は小さなエリア・低高度・低速でテストしてください
- 緊急時は Ctrl+C で安全停止（着陸試行）します
- ディスアーム検出時も自動停止します
- モード変更（Guided→他モード）検出時も自動停止します
