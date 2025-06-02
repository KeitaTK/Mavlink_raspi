#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
monitor_all_statustext.py

Pixhawk から飛んでくる STATUSTEXT メッセージを全てモニタリングして
シリアルポート上に表示するスクリプト
"""

import time
from pymavlink import mavutil

# 接続ポートとボーレートを環境に合わせて変更してください
PORT = '/dev/ttyAMA0'   # TELEM1 UART
# PORT = '/dev/ttyACM0' # USB-CDC ACM
BAUD = 115200

def monitor_all_statustext():
    master = mavutil.mavlink_connection(PORT, baud=BAUD)
    master.wait_heartbeat()
    print("=== STATUSTEXT Monitor (All Messages) ===")
    print(f"Connected on {PORT} @ {BAUD}bps")
    print("Monitoring all STATUSTEXT messages...")
    print("Press Ctrl+C to stop")
    print("-" * 50)

    severity_names = [
        'EMERGENCY','ALERT','CRITICAL','ERROR',
        'WARNING','NOTICE','INFO','DEBUG'
    ]

    try:
        while True:
            msg = master.recv_match(type='STATUSTEXT', blocking=False)
            if msg and msg.text:
                # バイト列の場合は文字列にデコード
                text = msg.text.decode('utf-8', errors='ignore') \
                       if isinstance(msg.text, (bytes, bytearray)) \
                       else msg.text
                sev = msg.severity
                name = severity_names[sev] if sev < len(severity_names) else str(sev)
                print(f"[{name}] {text}")
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\nStopping monitor.")
    finally:
        master.close()

if __name__ == '__main__':
    monitor_all_statustext()

