#!/usr/bin/env python3
"""
TELEM1詳細診断 - ハートビート分析
"""

from pymavlink import mavutil
import time

def detailed_telem1_diagnosis():
    """TELEM1接続の詳細診断"""
    
    print("=== TELEM1 Detailed Diagnosis ===")
    
    try:
        # デバッグモード付きで接続
        master = mavutil.mavlink_connection('/dev/serial0', baud=115200)
        
        print("Listening for heartbeats...")
        heartbeat_count = 0
        start_time = time.time()
        
        while time.time() - start_time < 15 and heartbeat_count < 10:
            msg = master.recv_match(blocking=False)
            if msg:
                msg_type = msg.get_type()
                
                if msg_type == 'HEARTBEAT':
                    heartbeat_count += 1
                    print(f"\nHeartbeat #{heartbeat_count}:")
                    print(f"  Source System: {msg.get_srcSystem()}")
                    print(f"  Source Component: {msg.get_srcComponent()}")
                    print(f"  Target System: {master.target_system}")
                    print(f"  Target Component: {master.target_component}")
                    print(f"  Type: {msg.type}")
                    print(f"  Autopilot: {msg.autopilot}")
                    print(f"  Base Mode: {msg.base_mode}")
                    print(f"  System Status: {msg.system_status}")
                
                elif msg_type not in ['HEARTBEAT']:
                    print(f"Other message: {msg_type}")
            
            time.sleep(0.1)
        
        print(f"\nTotal heartbeats received: {heartbeat_count}")
        
        if heartbeat_count > 0:
            print("✅ TELEM1 is receiving heartbeats")
            if master.target_system == 0:
                print("⚠️ Target system is still 0 - This indicates a communication issue")
            else:
                print(f"✅ Target system correctly identified as {master.target_system}")
        else:
            print("❌ No heartbeats received on TELEM1")
        
        master.close()
        return heartbeat_count > 0
        
    except Exception as e:
        print(f"❌ Diagnosis failed: {e}")
        return False

if __name__ == "__main__":
    detailed_telem1_diagnosis()

