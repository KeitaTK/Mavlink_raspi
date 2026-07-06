#!/usr/bin/env python3
"""Pixhawk全パラメータをダンプ（mavlink-router経由UDP）"""

import socket, struct, time, sys
from pymavlink.dialects.v20 import ardupilotmega

OUTPUT = "params_dump.txt"
TARGET = ("127.0.0.1", 14550)

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

# UDPソケット（ランダムポートでバインド）
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("127.0.0.1", 0))
sock.settimeout(3)
print(f"Bound to 127.0.0.1:{sock.getsockname()[1]}")

# MAVLinkパーサー
mav = ardupilotmega.MAVLink(bytearray(), srcSystem=255, srcComponent=190)

# HEARTBEAT受信を待つ
print("Waiting for heartbeat...")
target_sys = None
deadline = time.time() + 10
while time.time() < deadline:
    try:
        data, addr = sock.recvfrom(4096)
        msgs = mav.parse_buffer(data)
        for msg in (msgs or []):
            if msg.get_type() == "HEARTBEAT":
                target_sys = msg.get_srcSystem()
                target_comp = msg.get_srcComponent()
                print(f"  Heartbeat from sys={target_sys} comp={target_comp}")
                break
        if target_sys:
            break
    except socket.timeout:
        continue

if not target_sys:
    print("ERROR: No heartbeat received")
    sys.exit(1)

# パラメータ要求
print("Requesting parameters...")
msg = mav.param_request_list_encode(target_sys, target_comp)
frame = msg.pack(mav)
sock.sendto(frame, TARGET)

params = {}
start = time.time()

while time.time() - start < 30:
    try:
        data, addr = sock.recvfrom(4096)
        msgs = mav.parse_buffer(data)
        for msg in (msgs or []):
            if msg.get_type() == "PARAM_VALUE":
                pid = msg.param_id
                if isinstance(pid, bytes):
                    pid = pid.decode("utf-8", errors="replace").rstrip("\x00")
                pid = pid.strip()
                params[pid] = msg.param_value
                if len(params) % 100 == 0:
                    print(f"  {len(params)} params...")
                if hasattr(msg, "param_count") and msg.param_count > 0:
                    if len(params) >= msg.param_count:
                        print(f"  All {msg.param_count} received!")
                        break
        if hasattr(msg, "param_count") and msg.param_count > 0 and len(params) >= msg.param_count:
            break
    except socket.timeout:
        if params:
            break  # 少しでも取れたらOK
    if params and time.time() - start > 10:
        break

sock.close()

print(f"\nTotal: {len(params)} parameters")

with open(OUTPUT, "w") as f:
    f.write("=== ArduPilot Parameters ===\n")
    f.write(f"Time: {time.strftime('%Y-%m-%d %H:%M:%S')}\n\n")
    f.write("=== GPS/RTK/Serial ===\n")
    for key in KEYS:
        f.write(f"{key:25s} = {params.get(key, 'N/A')}\n")
    f.write("\n=== All Parameters ===\n")
    for key in sorted(params.keys()):
        f.write(f"{key:25s} = {params[key]}\n")

print(f"Saved to {OUTPUT}")
print("\n=== Key Parameters ===")
for key in KEYS:
    print(f"  {key:25s} = {params.get(key, 'N/A')}")
