#!/usr/bin/env python3
"""
Pixhawk System ID 確認・設定スクリプト
"""

from pymavlink import mavutil
import time

def check_and_fix_system_id():
    """System ID確認と修正"""
    
    print("=== System ID Diagnosis ===")
    
    try:
        # USB経由で接続（設定変更のため）
        print("Connecting via USB for parameter check...")
        master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
        master.wait_heartbeat()
        
        print(f"Current System ID: {master.target_system}")
        print(f"Current Component ID: {master.target_component}")
        
        # SYSID_THISMAVパラメータ確認
        print("\nChecking SYSID_THISMAV parameter...")
        master.mav.param_request_read_send(
            master.target_system,
            master.target_component,
            'SYSID_THISMAV'.encode('utf-8'),
            -1
        )
        
        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=5)
        if msg:
            current_sysid = msg.param_value
            print(f"SYSID_THISMAV = {current_sysid}")
            
            if current_sysid == 0:
                print("❌ System ID is 0 - This needs to be fixed!")
                
                # System IDを1に設定
                print("Setting SYSID_THISMAV to 1...")
                master.mav.param_set_send(
                    master.target_system,
                    master.target_component,
                    'SYSID_THISMAV'.encode('utf-8'),
                    1,
                    mavutil.mavlink.MAV_PARAM_TYPE_INT8
                )
                
                # 確認
                confirm_msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=5)
                if confirm_msg:
                    print(f"✅ SYSID_THISMAV set to {confirm_msg.param_value}")
                    
                    # パラメータ保存
                    print("Saving parameters...")
                    master.mav.command_long_send(
                        master.target_system,
                        master.target_component,
                        mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE,
                        0,
                        1, 0, 0, 0, 0, 0, 0
                    )
                    
                    print("✅ System ID fixed! Please reboot Pixhawk.")
                    return True
            else:
                print(f"✅ System ID is correctly set to {current_sysid}")
                return True
        else:
            print("❌ Failed to read SYSID_THISMAV")
            return False
            
    except Exception as e:
        print(f"❌ Error: {e}")
        return False
    finally:
        master.close()

if __name__ == "__main__":
    check_and_fix_system_id()
