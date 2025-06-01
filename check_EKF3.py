#!/usr/bin/env python3
"""
EKFリセット後のモーションキャプチャ継続確認
"""

from pymavlink import mavutil
import time

def check_post_reset():
    """リセット後の状態確認"""
    
    master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
    master.wait_heartbeat()
    
    print("=== Post-Reset Status Check ===")
    print("Checking if motion capture data is still flowing...")
    
    start_time = time.time()
    message_counts = {}
    
    while time.time() - start_time < 10:
        msg = master.recv_match(blocking=False)
        if msg:
            msg_type = msg.get_type()
            message_counts[msg_type] = message_counts.get(msg_type, 0) + 1
        time.sleep(0.01)
    
    print(f"\nMessages received in 10 seconds:")
    for msg_type, count in sorted(message_counts.items()):
        if count > 0:
            rate = count / 10.0
            print(f"  {msg_type}: {count} messages ({rate:.1f} Hz)")
    
    if 'ATT_POS_MOCAP' in message_counts:
        mocap_rate = message_counts['ATT_POS_MOCAP'] / 10.0
        print(f"\n✅ Motion capture data flowing at {mocap_rate:.1f} Hz")
        return True
    else:
        print(f"\n❌ No motion capture data - check TELEM1 connection")
        return False
    
    master.close()

if __name__ == "__main__":
    check_post_reset()
