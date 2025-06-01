#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
send_pos2.py

UDP（モーションキャプチャ）から位置・姿勢データを取得し、
20HzでPixhawk TELEM1（またはUSB）にMAVLink ATT_POS_MOCAPメッセージを送信するスクリプト。
Ctrl+CでソケットとMAVLink接続をクリーンに閉じます。
"""

import socket
import pickle
import signal
import sys
import time
from pymavlink import mavutil

#---- クラス定義 ----
class FrequencyController:
    """一定周波数で送信を制御する"""
    def __init__(self, target_hz=20):
        self.interval = 1.0 / target_hz
        self.last_send_time = 0.0

    def should_send(self):
        now = time.time()
        if now - self.last_send_time >= self.interval:
            self.last_send_time = now
            return True
        return False

class LatestDataBuffer:
    """最新のUDPパケットのみを保持するバッファ"""
    def __init__(self):
        self.latest = None
        self.count = 0

    def update(self, data):
        self.latest = data
        self.count += 1

    def get_latest(self):
        return self.latest

    def get_count(self):
        return self.count

#---- ユーティリティ関数 ----
def get_latest_udp_data(sock):
    """
    ノンブロッキングでUDPソケットを読み、
    最後に受け取ったパケットのデータのみを返す
    """
    sock.setblocking(False)
    newest = None
    read_count = 0

    while True:
        try:
            data, addr = sock.recvfrom(1024)
            newest = data
            read_count += 1
        except socket.error as e:
            # EWOULDBLOCK/EAGAIN なら読み終わり
            err = e.errno if hasattr(e, 'errno') else None
            if err in (socket.errno.EWOULDBLOCK, socket.errno.EAGAIN):
                break
            else:
                raise
    return newest, read_count

def signal_handler(signum, frame):
    """Ctrl+C ハンドラー: ソケットとMAVLinkをクリーンに閉じて終了"""
    print("\nCtrl+C pressed! Cleaning up and exiting...")
    try:
        sock.close()
        print("UDP socket closed")
    except:
        pass
    try:
        master.close()
        print("MAVLink connection closed")
    except:
        pass
    sys.exit(0)

#---- メイン処理 ----
def receiver_with_20hz_control():
    global sock, master

    # SIGINTハンドラー登録
    signal.signal(signal.SIGINT, signal_handler)

    # UDPソケット作成
    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(('', 15769))
        print("✓ UDP socket bound to port 15769")
    except Exception as e:
        print(f"✗ UDP socket setup failed: {e}")
        sys.exit(1)

    # MAVLink接続
    print("Setting up MAVLink connection...")
    try:
        # USB接続の場合:
        # master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
        # TELEM1（UART）接続の場合:
        master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
        master.wait_heartbeat(timeout=10)
        print(f"✓ MAVLink connected (sysid={master.target_system})")
    except Exception as e:
        print(f"✗ MAVLink connection failed: {e}")
        sock.close()
        sys.exit(1)

    # 送信周波数制御＆最新データバッファ
    freq_ctrl = FrequencyController(target_hz=20)
    buffer    = LatestDataBuffer()

    print("Receiving 50Hz UDP data, sending at 20Hz via MAVLink...")
    print("Press Ctrl+C to stop")
    print("-" * 70)

    total_packets   = 0
    skipped_packets = 0
    send_count      = 0

    try:
        while True:
            # UDPから最新データを取得
            data_bytes, n_read = get_latest_udp_data(sock)
            if data_bytes:
                # pickleデータを復元
                obj = pickle.loads(data_bytes)
                pos  = obj['position']     # [x, y, z]
                quat = obj['quaternion']   # [w, x, y, z]

                buffer.update({'pos': pos, 'quat': quat})
                total_packets += 1
                if n_read > 1:
                    skipped_packets += (n_read - 1)

                # 周波数制御で送信
                if freq_ctrl.should_send():
                    latest = buffer.get_latest()
                    # MAVLink送信
                    ts_usec = int(time.time() * 1e6)
                    master.mav.att_pos_mocap_send(
                        ts_usec,
                        latest['quat'],
                        latest['pos'][0],
                        latest['pos'][1],
                        latest['pos'][2]
                    )
                    send_count += 1
                    print(f"[{send_count:03d}] Sent Pos: ({latest['pos'][0]:+6.2f}, "
                          f"{latest['pos'][1]:+6.2f}, {latest['pos'][2]:+6.2f}) | "
                          f"UDP recv: {total_packets} pkts | skipped: {skipped_packets}")

            # CPU負荷低減
            time.sleep(0.001)

    except Exception as e:
        print(f"Main loop error: {e}")

    finally:
        # 終了処理
        sock.close()
        if master:
            master.close()
        print("\n=== Statistics ===")
        print(f"  Total UDP packets received: {total_packets}")
        print(f"  Total packets skipped:      {skipped_packets}")
        print(f"  MAVLink messages sent:      {send_count}")
        if total_packets>0:
            print(f"  Effective send rate:        {send_count/(total_packets/50.0):.1f} Hz")

# エントリーポイント
if __name__ == '__main__':
    receiver_with_20hz_control()
