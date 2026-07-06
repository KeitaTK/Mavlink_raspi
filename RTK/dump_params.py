#!/usr/bin/env python3
"""Pixhawk全パラメータをダンプ（シリアル直）

前提: sudo systemctl stop mavlink-router でUARTを解放してから実行。
終わったら sudo systemctl start mavlink-router で再開。
"""

import time, sys
from pymavlink import mavutil

OUTPUT = "params_dump.txt"

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

print("Connecting /dev/ttyAMA0 @ 115200...")
mav = mavutil.mavlink_connection("/dev/ttyAMA0", baud=115200)

print("Waiting for heartbeat...")
mav.wait_heartbeat()
print(f"Heartbeat from sys={mav.target_system}")

print("Requesting all parameters...")
mav.mav.param_request_list_send(mav.target_system, mav.target_component)

params = {}
start = time.time()

while time.time() - start < 30:
    msg = mav.recv_match(type="PARAM_VALUE", blocking=True, timeout=2)
    if msg is None:
        if params:
            break
        continue
    pid = msg.param_id
    if isinstance(pid, bytes):
        pid = pid.decode("utf-8", errors="replace").rstrip("\x00")
    pid = pid.strip()
    params[pid] = msg.param_value
    if len(params) % 100 == 0:
        print(f"  {len(params)}...")
    if hasattr(msg, "param_count") and msg.param_count > 0 and len(params) >= msg.param_count:
        print(f"  All {msg.param_count} received!")
        break

print(f"\nTotal: {len(params)} parameters")

with open(OUTPUT, "w") as f:
    f.write(f"=== ArduPilot Parameters ({time.strftime('%Y-%m-%d %H:%M:%S')}) ===\n\n")
    f.write("=== GPS/RTK/Serial ===\n")
    for key in KEYS:
        f.write(f"{key:25s} = {params.get(key, 'N/A')}\n")
    f.write("\n=== All ===\n")
    for key in sorted(params.keys()):
        f.write(f"{key:25s} = {params[key]}\n")

print(f"\nSaved to {OUTPUT}")
print("\n=== Key Parameters ===")
for key in KEYS:
    print(f"  {key:25s} = {params.get(key, 'N/A')}")
