#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
setup_ekf_mocap_only.py

EKF3 を Motion Capture + 内部 IMU のみで動作させるための
パラメータ設定スクリプト

- GPS 無効化
- コンパス無効化
- EKF3 シングル IMU + ExternalNav
- イノベーションゲート調整
- EEPROM 保存 → 再起動
"""

import time
from pymavlink import mavutil

def setup_ekf_mocap_only(port='/dev/ttyAMA0', baud=115200):
    master = mavutil.mavlink_connection(port, baud=baud)
    master.wait_heartbeat()
    print(f"Connected to system {master.target_system}, component {master.target_component}")

    params = {
        # --- 基本 EKF 設定 ---
        'AHRS_EKF_TYPE':   3.0,   # EKF3
        # 'EK2_ENABLE':      0.0,   # EKF2 無効
        'EK3_ENABLE':      1.0,   # EKF3 有効
        'EK3_IMU_MASK':    1.0,   # シングル IMU

        # --- 外部位置データ (Motion Capture) Source ---
        'EK3_SRC1_POSXY':  6.0,   # ExternalNav (水平位置)
        'EK3_SRC1_VELXY':  0.0,   # None (速度は推定)
        'EK3_SRC1_POSZ':   6.0,   # ExternalNav (垂直位置)
        'EK3_SRC1_VELZ':   0.0,   # None
        'EK3_SRC1_YAW':    6.0,   # ExternalNav (ヨー角)

        # --- イノベーション ゲート ---
        'EK3_POS_I_GATE':  3.0,
        'EK3_VEL_I_GATE':  5.0,
        'EK3_HGT_I_GATE':  3.0,

        # --- GPS 無効化 (GPS 端末を無視) ---
        # 'GPS_TYPE':        0.0,   # GPS なし
        'GPS_AUTO_CONFIG': 0.0,   # GPS 自動設定無効
        # 'GPS_DELAY_MS':    0.0,   # GPS 遅延 0

        # --- コンパス無効化 ---
        'COMPASS_USE':     0.0,   # 内蔵コンパス OFF
        'COMPASS_USE2':    0.0,   # 外付コンパス2 OFF
        'COMPASS_USE3':    0.0,   # 外付コンパス3 OFF
    }

    print("\n=== Setting EKF3 parameters for Motion Capture only ===")
    for name, val in params.items():
        print(f" {name:20s} = {val}")
        master.mav.param_set_send(
            master.target_system,
            master.target_component,
            name.encode('utf-8'),
            val,
            mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        )
        # 確認
        resp = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
        if resp:
            pid = resp.param_id
            if isinstance(pid, bytes):
                pid = pid.decode('utf-8').rstrip('\x00')
            print(f"   → {pid} = {resp.param_value}")
        else:
            print(f"   ⚠️ Timeout on {name}")
        time.sleep(0.2)

    # EEPROM に保存
    print("\nSaving parameters to EEPROM…")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    time.sleep(1)

    # Pixhawk 再起動
    print("Rebooting Pixhawk to apply changes…")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    print(" → Reboot command sent. Please wait ~30s for reboot.")
    master.close()

if __name__ == "__main__":
    setup_ekf_mocap_only()
