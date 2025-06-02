#!/usr/bin/env python3
from pymavlink import mavutil
import time

def monitor_debug_messages():
# USB接続の場合
    print("USB接続通信")
    master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
    # telem1で接続するとき
    # print("tekem1通信接続")
    # master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
    master.wait_heartbeat()
    
    print("Monitoring STATUSTEXT messages...")
    print("Send ATT_POS_MOCAP data to see debug output")
    print("-" * 50)
    
    while True:
        msg = master.recv_match(type='STATUSTEXT', blocking=False)
        if msg:
            # ATT_POS_MOCAP関連のメッセージのみ表示
            if 'ATT_POS_MOCAP' in msg.text:
                severity = ['EMERGENCY', 'ALERT', 'CRITICAL', 'ERROR', 
                           'WARNING', 'NOTICE', 'INFO', 'DEBUG'][msg.severity]
                print(f"[{severity}] {msg.text}")
        
        time.sleep(0.01)

if __name__ == "__main__":
    monitor_debug_messages()
