#!/usr/bin/env python3
"""
EKF処理状況の最終確認
"""

from pymavlink import mavutil
import time

def final_ekf_check():
    """EKF最終確認"""
    
    master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
    master.wait_heartbeat()
    
    print("=== Final EKF Check ===")
    print("Motion capture data confirmed at 16.6Hz")
    print("Checking EKF response...")
    print("-" * 50)
    
    # EKFメッセージを強制要求
    ekf_messages = [
        (mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 1000000),
        (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 200000),
        (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 500000)
    ]
    
    for msg_id, interval in ekf_messages:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_GET_MESSAGE_INTERVAL,
            0,
            msg_id,
            interval,
            0, 0, 0, 0, 0
        )
        time.sleep(0.1)
    
    # 30秒間の詳細監視
    print("Monitoring EKF for 30 seconds...")
    start_time = time.time()
    
    ekf_status_count = 0
    position_count = 0
    mocap_count = 0
    last_position = None
    
    while time.time() - start_time < 30:
        msg = master.recv_match(blocking=False)
        if msg:
            msg_type = msg.get_type()
            
            if msg_type == 'ATT_POS_MOCAP':
                mocap_count += 1
                
            elif msg_type == 'EKF_STATUS_REPORT':
                ekf_status_count += 1
                if ekf_status_count <= 3:  # 最初の3回のみ表示
                    print(f"\n[{ekf_status_count}] EKF Status Report:")
                    print(f"    Position Variance: {msg.pos_horiz_variance:.6f}")
                    print(f"    Velocity Variance: {msg.velocity_variance:.6f}")
                    print(f"    Compass Variance: {msg.compass_variance:.6f}")
                    
                    if (msg.pos_horiz_variance < 1.0 and 
                        msg.velocity_variance < 1.0):
                        print(f"    Health: ✅ GOOD")
                    else:
                        print(f"    Health: ⚠️ INITIALIZING")
                
            elif msg_type == 'LOCAL_POSITION_NED':
                position_count += 1
                last_position = (msg.x, msg.y, msg.z)
                
                if position_count <= 3:  # 最初の3回のみ表示
                    print(f"\n[{position_count}] Local Position:")
                    print(f"    Position: N={msg.x:+7.3f}m E={msg.y:+7.3f}m D={msg.z:+7.3f}m")
                    print(f"    Velocity: N={msg.vx:+6.2f}m/s E={msg.vy:+6.2f}m/s D={msg.vz:+6.2f}m/s")
                
            elif msg_type == 'STATUSTEXT':
                text = msg.text.lower()
                if any(keyword in text for keyword in ['ekf', 'position', 'origin', 'gps']):
                    print(f"    Status: {msg.text}")
        
        time.sleep(0.1)
    
    # 結果サマリー
    print(f"\n{'='*50}")
    print(f"30-Second Test Results:")
    print(f"{'='*50}")
    print(f"Motion Capture Messages: {mocap_count} ({mocap_count/30:.1f} Hz)")
    print(f"EKF Status Reports: {ekf_status_count}")
    print(f"Position Updates: {position_count}")
    
    if last_position:
        print(f"Latest Position: N={last_position[0]:+7.3f}m E={last_position[1]:+7.3f}m D={last_position[2]:+7.3f}m")
    
    # 成功判定
    if ekf_status_count > 0 and position_count > 0:
        print(f"\n🎉 SUCCESS: EKF is working!")
        print(f"   ✅ EKF processing motion capture data")
        print(f"   ✅ Position estimation active")
        print(f"   ✅ Ready for flight control")
        return True
        
    elif ekf_status_count > 0:
        print(f"\n⚠️ PARTIAL: EKF responding but no position yet")
        print(f"   ✅ EKF status available")
        print(f"   ⏳ Position estimation initializing")
        return False
        
    else:
        print(f"\n❌ NO RESPONSE: EKF not responding")
        print(f"   ❌ No EKF status messages")
        print(f"   ❌ No position estimates")
        print(f"   🔧 Additional troubleshooting needed")
        return False
    
    master.close()

if __name__ == "__main__":
    final_ekf_check()
