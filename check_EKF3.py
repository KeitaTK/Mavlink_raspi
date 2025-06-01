#!/usr/bin/env python3
"""
EKF + モーションキャプチャ統合テスト
"""

from pymavlink import mavutil
import socket
import pickle
import time

def integrated_ekf_mocap_test():
    """統合テスト"""
    
    print("=== EKF + Motion Capture Integration Test ===")
    
    # TELEM1接続
    # master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
    master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
    master.wait_heartbeat()
    print("✅ TELEM1 connected")
    
    # UDP setup
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', 15769))
    sock.setblocking(False)
    print("✅ UDP socket ready")
    
    # EKFステータス要求
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_GET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT,
        1000000,  # 1Hz
        0, 0, 0, 0, 0
    )
    
    print("\nWaiting for motion capture data...")
    print("Start Motion Capture data transmission now!")
    print("-" * 50)
    
    mocap_count = 0
    ekf_reports = 0
    start_time = time.time()
    
    try:
        while time.time() - start_time < 30:  # 30秒テスト
            # モーションキャプチャデータ確認
            try:
                data_bytes, addr = sock.recvfrom(1024)
                if data_bytes:
                    mocap_count += 1
                    data = pickle.loads(data_bytes)
                    pos = data['position']
                    
                    # ATT_POS_MOCAP送信
                    time_usec = int(time.time() * 1000000)
                    master.mav.att_pos_mocap_send(
                        time_usec,
                        data['quaternion'],
                        pos[0], pos[1], pos[2]
                    )
                    
                    if mocap_count % 50 == 1:
                        print(f"[{mocap_count:03d}] MOCAP → ArduPilot: ({pos[0]:+6.2f}, {pos[1]:+6.2f}, {pos[2]:+6.2f})")
            except:
                pass
            
            # EKFステータス確認
            msg = master.recv_match(blocking=False)
            if msg and msg.get_type() == 'EKF_STATUS_REPORT':
                ekf_reports += 1
                print(f"EKF Health: Pos={msg.pos_horiz_variance:.4f}, Vel={msg.velocity_variance:.4f}")
            
            time.sleep(0.02)  # 50Hz
            
    except KeyboardInterrupt:
        print("\nTest stopped")
    
    print(f"\n=== Test Results ===")
    print(f"MOCAP packets: {mocap_count}")
    print(f"EKF reports: {ekf_reports}")
    
    if mocap_count > 0:
        print("✅ Motion capture data integration working!")
    else:
        print("❌ No motion capture data received")
    
    sock.close()
    master.close()

if __name__ == "__main__":
    integrated_ekf_mocap_test()
