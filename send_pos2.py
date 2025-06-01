#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from pymavlink import mavutil
import socket           # ← 忘れずに追加
import pickle
import signal
import sys
import time

# 以下クラス定義は省略 …

def receiver_with_20hz_control():
    global sock, master

    # SIGINT ハンドラー設定
    signal.signal(signal.SIGINT, signal_handler)

    # 1) UDP ソケット作成＆確認
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('', 15769))
        print("✓ UDP socket bound to port 15769")
    except Exception as e:
        print(f"✗ UDP socket setup failed: {e}")
        sys.exit(1)

    # 2) MAVLink 接続確立＆ハートビート待ち
    print("Setting up MAVLink connection...")
    try:
        # USB→ '/dev/ttyACM0' / TELEM1→ '/dev/ttyAMA0' に切り替え
        master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
        master.wait_heartbeat(timeout=10)
        print(f"✓ MAVLink connected (sysid={master.target_system})")
    except Exception as e:
        print(f"✗ MAVLink connection failed: {e}")
        sock.close()
        sys.exit(1)

    # --- ここから送信ループへ進む ---
    freq_controller = FrequencyController(target_hz=20)
    data_buffer = LatestDataBuffer()

    print("Receiving 50Hz Motive data, sending at 20Hz…")
    print("Press Ctrl+C to stop")
    print("-" * 70)

    packet_count = 0
    mavlink_send_count = 0
    skipped_packets = 0

    try:
        while True:
            # 最新 UDP データ取得
            newest_data, packets_read = get_latest_udp_data(sock)

            if newest_data:
                data = pickle.loads(newest_data)
                pos  = data['position']
                quat = data['quaternion']

                data_buffer.update({'pos': pos, 'quat': quat})
                packet_count += 1
                if packets_read > 1:
                    skipped_packets += packets_read - 1

                # 20Hz 制御で送信
                if freq_controller.should_send():
                    latest = data_buffer.get_latest()
                    time_usec = int(time.time() * 1e6)
                    master.mav.att_pos_mocap_send(
                        time_usec,
                        latest['quat'],
                        latest['pos'][0],
                        latest['pos'][1],
                        latest['pos'][2]
                    )
                    mavlink_send_count += 1
                    print(f"[{mavlink_send_count:03d}] Pos: {latest['pos']} | "
                          f"RX: {packet_count} pkts | Skipped: {skipped_packets}")

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\nStopping…")

    finally:
        sock.close()
        if master:
            master.close()
        print(f"\nStatistics:")
        print(f"  UDP packets received: {packet_count}")
        print(f"  Packets skipped:      {skipped_packets}")
        print(f"  MAVLink sent:         {mavlink_send_count}")
        print(f"  Effective rate:       {mavlink_send_count/(packet_count/50):.1f} Hz")

if __name__ == "__main__":
    receiver_with_20hz_control()
