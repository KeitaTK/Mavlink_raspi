#!/usr/bin/env python3
"""
Ctrl+C のみで終了する簡易版
"""

import socket
import pickle
import signal
import sys

def signal_handler(signum, frame):
    """Ctrl+C ハンドラー"""
    print("\nCtrl+C pressed! Closing socket and exiting...")
    if 'sock' in globals():
        sock.close()
        print("Socket closed")
    sys.exit(0)

def simple_receiver_with_ctrl_c():
    global sock
    
    # SIGINT (Ctrl+C) ハンドラー設定
    signal.signal(signal.SIGINT, signal_handler)
    
    # UDPソケット作成
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('', 15769))
    
    print("Waiting for Motive data on port 15769...")
    print("Press Ctrl+C to stop")
    
    packet_count = 0
    
    try:
        while True:
            try:
                # タイムアウト付き受信
                sock.settimeout(1.0)
                data_bytes, sender_addr = sock.recvfrom(1024)
                packet_count += 1
                
                # データ処理
                data = pickle.loads(data_bytes)
                pos = data['position']
                quat = data['quaternion']
                
                print(f"#{packet_count:04d} | Pos: ({pos[0]:+6.2f}, {pos[1]:+6.2f}, {pos[2]:+6.2f})")
                
            except socket.timeout:
                continue  # タイムアウト時は継続
            except Exception as e:
                print(f"Data error: {e}")
                
    except Exception as e:
        print(f"Error: {e}")
    finally:
        sock.close()
        print(f"Total packets: {packet_count}")

if __name__ == "__main__":
    simple_receiver_with_ctrl_c()
