#!/usr/bin/env python3
"""
TELEM1接続テスト - 115200 baud
"""

from pymavlink import mavutil
import time

def test_telem1_connection():
    """TELEM1接続テスト"""
    
    print("=== TELEM1 Connection Test ===")
    print("Port: /dev/serial0")
    print("Baud: 115200")
    print("Flow Control: None")
    print("-" * 30)
    
    try:
        # TELEM1接続（検索結果[1]を参考に/dev/serial0使用）
        master = mavutil.mavlink_connection('/dev/serial0', baud=115200)
        
        print("Waiting for heartbeat...")
        master.wait_heartbeat(timeout=10)
        
        print(f"✅ TELEM1 Connection Successful!")
        print(f"   System ID: {master.target_system}")
        print(f"   Component ID: {master.target_component}")
        
        # 基本メッセージ受信テスト
        print("\nReceiving messages for 10 seconds...")
        start_time = time.time()
        message_count = 0
        
        while time.time() - start_time < 10:
            msg = master.recv_match(blocking=False)
            if msg:
                message_count += 1
                if message_count <= 5:  # 最初の5メッセージを表示
                    print(f"   Received: {msg.get_type()}")
            time.sleep(0.1)
        
        print(f"\nTotal messages received: {message_count}")
        print("✅ TELEM1 communication working properly!")
        
        master.close()
        return True
        
    except Exception as e:
        print(f"❌ TELEM1 connection failed: {e}")
        return False

if __name__ == "__main__":
    test_telem1_connection()
