#!/usr/bin/env python3
"""
TELEM1接続テスト - System ID問題対応版
"""

from pymavlink import mavutil
import time

def test_telem1_with_system_id():
    """System ID指定でTELEM1テスト"""
    
    print("=== TELEM1 Connection Test (System ID Fix) ===")
    
    try:
        # System ID 1を明示的に指定
        master = mavutil.mavlink_connection(
            '/dev/serial0', 
            baud=115200,
            source_system=255,  # 自分のSystem ID
            source_component=0
        )
        
        print("Waiting for heartbeat...")
        master.wait_heartbeat(timeout=10)
        
        print(f"✅ Connection successful!")
        print(f"   Target System ID: {master.target_system}")
        print(f"   Target Component ID: {master.target_component}")
        
        # メッセージストリーム要求
        print("\nRequesting message streams...")
        
        # 複数のメッセージを要求
        message_requests = [
            (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 1),
            (mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT, 1),
            (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 10),
            (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10)
        ]
        
        for msg_id, rate_hz in message_requests:
            master.mav.command_long_send(
                master.target_system,
                master.target_component,
                mavutil.mavlink.MAV_CMD_GET_MESSAGE_INTERVAL,
                0,
                msg_id,
                1000000 // rate_hz,  # マイクロ秒間隔
                0, 0, 0, 0, 0
            )
            time.sleep(0.1)
        
        # メッセージ受信テスト
        print("\nListening for messages (15 seconds)...")
        start_time = time.time()
        message_types = {}
        
        while time.time() - start_time < 15:
            msg = master.recv_match(blocking=False)
            if msg:
                msg_type = msg.get_type()
                message_types[msg_type] = message_types.get(msg_type, 0) + 1
            time.sleep(0.1)
        
        print(f"\nReceived message types:")
        for msg_type, count in message_types.items():
            print(f"   {msg_type}: {count} messages")
        
        total_messages = sum(message_types.values())
        print(f"\nTotal messages: {total_messages}")
        
        if total_messages > 5:
            print("✅ TELEM1 communication is working properly!")
        else:
            print("⚠️ Limited messages received - System ID issue may persist")
        
        master.close()
        return total_messages > 5
        
    except Exception as e:
        print(f"❌ Connection failed: {e}")
        return False

if __name__ == "__main__":
    test_telem1_with_system_id()
