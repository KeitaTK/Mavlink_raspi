#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from pymavlink import mavutil
import time

PORT    = '/dev/ttyACM0'
BAUD    = 115200
DURATION = 10.0

def main():
    # 1. 接続＆ハートビート待ち
    master = mavutil.mavlink_connection(PORT, baud=BAUD)
    master.wait_heartbeat()
    print(f"Heartbeat from sysid={master.target_system}")

    # 2. データストリームを要求
    #    RAW IMU, SYS_STATUS/EKF, ATTITUDE, LOCAL_POSITION_NED,
    #    plus HIGHRES_IMU をそれぞれ request_data_stream_send ではなく 
    #    MAVLink2 の GET_MESSAGE_INTERVAL で要求します。
    def set_interval(msg_id, hz):
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_GET_MESSAGE_INTERVAL,
            0,
            msg_id,
            int(1e6/hz),  # マイクロ秒間隔
            0,0,0,0,0
        )
        time.sleep(0.05)

    # 高速 IMU（HIGHRES_IMU）
    set_interval(mavutil.mavlink.MAVLINK_MSG_ID_HIGHRES_IMU,     50)
    # EKF 健全性レポート
    set_interval(mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT,2)
    # 姿勢推定
    set_interval(mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,        10)
    # システム状態
    set_interval(mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,      1)
    # ローカル位置（EKF 推定）
    set_interval(mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,5)

    # 3. メッセージ受信＆カウント
    print(f"\nListening for {DURATION}s…")
    counts = {}
    start = time.time()
    while time.time() - start < DURATION:
        msg = master.recv_match(blocking=False)
        if not msg:
            time.sleep(0.01)
            continue
        t = msg.get_type()
        counts[t] = counts.get(t, 0) + 1

    # 4. 結果表示
    print("\n=== Message Counts ===")
    for m, c in sorted(counts.items()):
        print(f"{m:25s}: {c:3d} msgs ({c/DURATION:.1f} Hz)")

    # 5. 判定
    if counts.get('HIGHRES_IMU',0)==0:
        print("❌ HIGHRES_IMU が受信できていません")
    if counts.get('ATTITUDE',0)==0:
        print("❌ ATTITUDE が受信できていません")
    if counts.get('SYS_STATUS',0)==0:
        print("❌ SYS_STATUS が受信できていません")
    if counts.get('EKF_STATUS_REPORT',0)==0:
        print("❌ EKF_STATUS_REPORT が受信できていません")
    if counts.get('LOCAL_POSITION_NED',0)==0:
        print("❌ LOCAL_POSITION_NED が受信できていません")
    if all(counts.get(k,0)>0 for k in (
        'HIGHRES_IMU','ATTITUDE','SYS_STATUS','EKF_STATUS_REPORT','LOCAL_POSITION_NED')):
        print("✅ EKF～位置推定パイプラインが正常に動作しています")

    master.close()

if __name__ == '__main__':
    main()

