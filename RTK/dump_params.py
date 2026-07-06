#!/usr/bin/env python3
"""Pixhawk全パラメータをダンプ（Mavlink_raspi/.venv で実行）"""

import time, sys
from pymavlink import mavutil

PORT = "udp:127.0.0.1:14550"
OUTPUT = "params_dump.txt"

# GPS/RTCM関連の重要パラメータ
KEYS = [
    "GPS_TYPE", "GPS_TYPE2", "GPS_AUTO_CONFIG", "GPS_AUTO_SWITCH",
    "GPS_INJECT_TO", "GPS_DRV_OPTIONS",
    "SERIAL1_PROTOCOL", "SERIAL1_BAUD",
    "SERIAL2_PROTOCOL", "SERIAL2_BAUD",
    "SERIAL3_PROTOCOL", "SERIAL3_BAUD",
    "SERIAL4_PROTOCOL", "SERIAL4_BAUD",
    "SERIAL5_PROTOCOL", "SERIAL5_BAUD",
    "CAN_P1_DRIVER", "CAN_P2_DRIVER",
    "EK2_ENABLE", "EK3_ENABLE", "EK3_SRC1_POSXY", "EK3_SRC1_VELXY",
    "AHRS_EKF_TYPE",
]

print(f"Connecting via {PORT} ...")
mav = mavutil.mavlink_connection(PORT)

# 接続待ち
print("Waiting for heartbeat...")
mav.wait_heartbeat()
print(f"Heartbeat from system {mav.target_system}")

# REQUEST_DATA_STREAM でテレメトリ抑制（パラメータ取得の帯域確保のため）
mav.mav.request_data_stream_send(
    mav.target_system, mav.target_component,
    0, 0, 1  # all streams off
)

# 全パラメータ取得
print("Requesting all parameters...")
mav.mav.param_request_list_send(mav.target_system, mav.target_component)

params = {}
start = time.time()
timeout = 30

with open(OUTPUT, "w") as f:
    f.write("=== ArduPilot Parameters ===\n")
    f.write(f"Timestamp: {time.strftime('%Y-%m-%d %H:%M:%S')}\n\n")

    while time.time() - start < timeout:
        msg = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=1)
        if msg is None:
            continue

        # Decode param_id
        pid = msg.param_id
        if isinstance(pid, bytes):
            pid = pid.decode("utf-8", errors="replace").rstrip("\x00")
        pid = pid.strip()

        params[pid] = msg.param_value

        # 進捗表示
        if len(params) % 50 == 0:
            print(f"  Received {len(params)} parameters...")

        # 全パラメータ受信完了判定（PARAM_VALUE の count フィールドで判定）
        if hasattr(msg, "param_count") and msg.param_count > 0:
            if len(params) >= msg.param_count:
                print(f"  All {msg.param_count} parameters received!")
                break

    print(f"\nTotal: {len(params)} parameters received")

    # GPS/RTK 関連を先頭に
    f.write("=== GPS/RTK/Serial 関連 ===\n")
    for key in KEYS:
        val = params.get(key, "N/A")
        f.write(f"{key:25s} = {val}\n")

    f.write("\n=== 全パラメータ（アルファベット順） ===\n")
    for key in sorted(params.keys()):
        f.write(f"{key:25s} = {params[key]}\n")

print(f"\nSaved to {OUTPUT}")
print(f"重要パラメータ:")
for key in KEYS:
    val = params.get(key, "N/A")
    print(f"  {key:25s} = {val}")
