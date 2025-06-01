#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
diagnose_full_ekf.py

EKF が自己位置推定に必要な各種メッセージを
正しく受信・処理しているかを10秒間で診断します。
"""

import time
from pymavlink import mavutil

# 設定
PORT     = '/dev/ttyAMA0'
BAUD     = 115200
DURATION = 10.0  # 監視時間（秒）

def main():
    master = mavutil.mavlink_connection(PORT, baud=BAUD)
    master.wait_heartbeat()
    print(f"Heartbeat from sysid={master.target_system} compid={master.target_component}\n")

    # ストリーム要求関数
    def req_stream(msg_id, hz):
        master.mav.command_long_send(
            master.target_system, master.target_component,
            mavutil.mavlink.MAV_CMD_GET_MESSAGE_INTERVAL,
            0, msg_id, int(1e6/hz), 0, 0, 0, 0, 0
        )

    # 必要なメッセージストリームをリクエスト
    print("Requesting streams…")
    req_stream(mavutil.mavlink.MAVLINK_MSG_ID_HIGHRES_IMU,    50)  # 生 IMU データ
    req_stream(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,       10)  # 姿勢
    req_stream(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,      1)  # センサー状態
    req_stream(mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT,1) # EKF 健全性
    req_stream(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,5) # ローカル位置
    time.sleep(0.2)

    print(f"\nListening for {DURATION:.1f}s…\n")
    start = time.time()
    counts = {}

    while time.time() - start < DURATION:
        msg = master.recv_match(blocking=False)
        if not msg:
            time.sleep(0.01)
            continue

        t = msg.get_type()
        counts[t] = counts.get(t, 0) + 1

        # 最初の1件を表示
        if counts[t] == 1:
            if t == 'HIGHRES_IMU':
                print(f"[{t}] accel=({msg.xacc:.2f},{msg.yacc:.2f},{msg.zacc:.2f}) gyro=({msg.xgyro:.2f},{msg.ygyro:.2f},{msg.zgyro:.2f})")
            elif t == 'ATTITUDE':
                print(f"[{t}] roll={msg.roll:.3f} pitch={msg.pitch:.3f} yaw={msg.yaw:.3f}")
            elif t == 'SYS_STATUS':
                print(f"[{t}] sensors_present=0x{msg.onboard_control_sensors_present:08X} enabled=0x{msg.onboard_control_sensors_enabled:08X}")
            elif t == 'EKF_STATUS_REPORT':
                print(f"[{t}] pos_var={msg.pos_horiz_variance:.6f} vel_var={msg.velocity_variance:.6f}")
            elif t == 'LOCAL_POSITION_NED':
                print(f"[{t}] N={msg.x:+6.3f} E={msg.y:+6.3f} D={msg.z:+6.3f}")

    # 結果サマリー
    print("\n" + "="*40)
    print("Message counts:")
    for t, c in sorted(counts.items()):
        print(f"  {t}: {c} msgs ({c/DURATION:.1f} Hz)")
    print("="*40)

    # 判定
    if counts.get('HIGHRES_IMU',0)==0:
        print("❌ No IMU data → check IMU connection/calibration")
    if counts.get('ATTITUDE',0)==0:
        print("❌ No ATTITUDE → EKF not running or no sensor data")
    if counts.get('SYS_STATUS',0)==0:
        print("❌ No SYS_STATUS → check system health")
    if counts.get('EKF_STATUS_REPORT',0)==0:
        print("❌ No EKF_STATUS_REPORT → EKF not initialized")
    if counts.get('LOCAL_POSITION_NED',0)==0:
        print("❌ No LOCAL_POSITION_NED → EKF not producing position")

    if (counts.get('HIGHRES_IMU',0)>0 and
        counts.get('ATTITUDE',0)>0 and
        counts.get('LOCAL_POSITION_NED',0)>0):
        print("\n✅ EKF is receiving IMU, computing attitude and producing position!")
    else:
        print("\n🔧 EKF pipeline incomplete—please address the missing stages above.")

    master.close()

if __name__ == '__main__':
    main()
