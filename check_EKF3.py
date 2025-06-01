#!/usr/bin/env python3
"""
EKF初期化状態確認
"""

from pymavlink import mavutil
import time

def check_ekf_initialization():
    """EKF初期化状態を詳しく確認"""
    
    master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
    master.wait_heartbeat()
    
    print("=== EKF Initialization Check ===")
    
    # 1. EKF関連パラメータ確認
    print("1. Checking EKF parameters...")
    ekf_params = {
        'AHRS_EKF_TYPE': 3,
        'EK3_ENABLE': 1,
        'EK3_SRC1_POSXY': 6,
        'EK3_SRC1_POSZ': 6,
        'EK3_SRC1_YAW': 6
    }
    
    for param, expected in ekf_params.items():
        master.mav.param_request_read_send(
            master.target_system,
            master.target_component,
            param.encode('utf-8'),
            -1
        )
        
        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
        if msg:
            param_name = str(msg.param_id).rstrip('\x00')
            actual = msg.param_value
            status = "✅" if actual == expected else "❌"
            print(f"  {status} {param_name} = {actual} (expected: {expected})")
        else:
            print(f"  ❌ {param} = TIMEOUT")
        
        time.sleep(0.2)
    
    # 2. EKFメッセージを強制リクエスト
    print("\n2. Forcing EKF message requests...")
    
    ekf_messages = [
        (mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, "EKF_STATUS_REPORT"),
        (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, "LOCAL_POSITION_NED"),
        (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, "GLOBAL_POSITION_INT")
    ]
    
    for msg_id, name in ekf_messages:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_GET_MESSAGE_INTERVAL,
            0,
            msg_id,
            500000,  # 2Hz (より頻繁に)
            0, 0, 0, 0, 0
        )
        print(f"  Requested {name} at 2Hz")
        time.sleep(0.2)
    
    # 3. システム状態詳細確認
    print("\n3. Checking system sensors...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_GET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,
        1000000,
        0, 0, 0, 0, 0
    )
    
    # 4. 20秒間の詳細監視
    print("\n4. Monitoring for 20 seconds...")
    start_time = time.time()
    message_counts = {}
    ekf_data_found = False
    
    while time.time() - start_time < 20:
        msg = master.recv_match(blocking=False)
        if msg:
            msg_type = msg.get_type()
            message_counts[msg_type] = message_counts.get(msg_type, 0) + 1
            
            if msg_type == 'SYS_STATUS':
                print(f"  System sensors enabled: 0x{msg.onboard_control_sensors_enabled:08X}")
                print(f"  System sensors health: 0x{msg.onboard_control_sensors_health:08X}")
                
            elif msg_type == 'EKF_STATUS_REPORT':
                ekf_data_found = True
                print(f"  ✅ EKF Status: pos_var={msg.pos_horiz_variance:.6f}, vel_var={msg.velocity_variance:.6f}")
                
            elif msg_type == 'LOCAL_POSITION_NED':
                ekf_data_found = True
                print(f"  ✅ Local Position: ({msg.x:.3f}, {msg.y:.3f}, {msg.z:.3f})")
                
            elif msg_type == 'STATUSTEXT':
                print(f"  Status: {msg.text}")
        
        time.sleep(0.1)
    
    print(f"\n=== Summary ===")
    print(f"Message types received:")
    for msg_type, count in sorted(message_counts.items()):
        print(f"  {msg_type}: {count}")
    
    print(f"\nEKF Data Found: {'✅' if ekf_data_found else '❌'}")
    
    master.close()
    return ekf_data_found

if __name__ == "__main__":
    check_ekf_initialization()
