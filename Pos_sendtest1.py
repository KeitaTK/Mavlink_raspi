# from pymavlink import mavutil
# import time

# # 接続確立
# master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)

# # カスタムメッセージが利用可能か確認
# if hasattr(master.mav, 'taki_pos_motive_send'):
#     print("Custom message available!")
#     master.mav.taki_pos_motive_send(
#         1.5,  # x_coord
#         2.3,  # y_coord
#         3.7   # z_coord
#     )
#     print("TAKI_POS_MOTIVE sent: x=1.5, y=2.3, z=3.7")
# else:
#     print("Custom message NOT available!")
#     # 利用可能なメソッドを確認
#     methods = [method for method in dir(master.mav) if 'send' in method.lower()]
#     print("Available send methods:", methods[:10])


from pymavlink import mavutil
import time

def test_custom_message_with_debug():
    """カスタムメッセージ送信とデバッグメッセージ受信のテスト"""
    
    # 接続確立
    master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
    print("Connecting to ArduPilot...")
    
    # ハートビート待ち
    master.wait_heartbeat()
    print("Heartbeat received, connection established!")
    
    # カスタムメッセージ送信テスト
    if hasattr(master.mav, 'taki_pos_motive_send'):
        print("✓ Custom message available!")
        
        # テストデータ送信
        test_data = [
            [1.5, 2.3, 3.7],
            [0.0, 0.0, -1.0],
            [2.5, -1.2, 0.8]
        ]
        
        for i, (x, y, z) in enumerate(test_data):
            print(f"\n--- Test {i+1}: Sending position ({x}, {y}, {z}) ---")
            
            # カスタムメッセージ送信
            master.mav.taki_pos_motive_send(x, y, z)
            print(f"TAKI_POS_MOTIVE sent: x={x}, y={y}, z={z}")
            
            # 応答とデバッグメッセージを監視
            receive_debug_messages(master, timeout=3)
            time.sleep(1)
            
    else:
        print("✗ Custom message NOT available!")
        print("Using standard message instead...")
        
        # 標準メッセージで代替送信
        master.mav.set_position_target_local_ned_send(
            int(time.time() * 1000) & 0xFFFFFFFF,
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            0b110111111000,
            1.5, 2.3, 3.7,  # position
            0, 0, 0,         # velocity
            0, 0, 0,         # acceleration
            0, 0             # yaw
        )
        print("Standard SET_POSITION_TARGET_LOCAL_NED sent")
        receive_debug_messages(master, timeout=5)

def receive_debug_messages(master, timeout=5):
    """デバッグメッセージを受信して表示"""
    print("Listening for debug messages...")
    
    start_time = time.time()
    received_count = 0
    
    while time.time() - start_time < timeout:
        msg = master.recv_match(blocking=False)
        if msg:
            mtype = msg.get_type()
            
            if mtype == 'STATUSTEXT':
                print(f"  🔍 DEBUG: {msg.text}")
                received_count += 1
                
            elif mtype == 'TAKI_POS_MOTIVE':
                print(f"  ✅ SUCCESS: TAKI_POS_MOTIVE received")
                print(f"     x={msg.x_coord}, y={msg.y_coord}, z={msg.z_coord}")
                received_count += 1
                
            elif mtype == 'HEARTBEAT':
                continue  # ハートビートは無視
                
            elif mtype in ['GLOBAL_POSITION_INT', 'ATTITUDE', 'SYS_STATUS']:
                continue  # 一般的なメッセージは無視
                
            else:
                print(f"  📨 Other: {mtype}")
        
        time.sleep(0.1)
    
    print(f"Debug monitoring finished. Received {received_count} relevant messages.")

def check_system_status(master):
    """システム状態確認"""
    print("\n--- System Status Check ---")
    
    # 利用可能な送信メソッドを確認
    send_methods = [method for method in dir(master.mav) if 'send' in method.lower()]
    print(f"Available send methods: {len(send_methods)}")
    
    # カスタムメッセージ関連をチェック
    custom_methods = [method for method in send_methods if 'taki' in method.lower()]
    if custom_methods:
        print(f"Custom TAKI methods: {custom_methods}")
    else:
        print("No custom TAKI methods found")
    
    # MAVLinkバージョン確認
    print(f"MAVLink version: {master.mav.protocol_version}")
    print(f"Target system: {master.target_system}, component: {master.target_component}")

# メイン実行
if __name__ == "__main__":
    try:
        check_system_status(mavutil.mavlink_connection('/dev/ttyACM0', baud=57600))
        test_custom_message_with_debug()
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"Error occurred: {e}")
        import traceback
        traceback.print_exc()
