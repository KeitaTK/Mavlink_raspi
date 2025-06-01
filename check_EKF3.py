#!/usr/bin/env python3
"""
基本メッセージ受信診断
"""

from pymavlink import mavutil
import time

def diagnose_basic_messages():
    """基本メッセージ受信診断"""
    
    master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
    master.wait_heartbeat()
    
    print("=== Basic Message Diagnosis ===")
    print("Listening for any messages for 10 seconds...")
    
    message_types = {}
    start_time = time.time()
    
    while time.time() - start_time < 10:
        msg = master.recv_match(blocking=False)
        if msg:
            msg_type = msg.get_type()
            message_types[msg_type] = message_types.get(msg_type, 0) + 1
        time.sleep(0.01)
    
    print(f"\nReceived message types:")
    for msg_type, count in sorted(message_types.items()):
        print(f"  {msg_type}: {count} messages")
    
    total = sum(message_types.values())
    print(f"\nTotal messages: {total}")
    
    if total == 0:
        print("❌ No messages received - check USB connection")
    elif 'HEARTBEAT' in message_types:
        print("✅ Basic communication working")
    
    master.close()

if __name__ == "__main__":
    diagnose_basic_messages()
