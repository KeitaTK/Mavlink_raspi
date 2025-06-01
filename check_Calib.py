def verify_calibration_status():
    """キャリブレーション状態確認"""
    
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
    master.wait_heartbeat()
    
    print("=== Calibration Status Check ===")
    
    # EKF健全性確認
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_GET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT,
        1000000,  # 1Hz
        0, 0, 0, 0, 0
    )
    
    print("Waiting for EKF status...")
    start_time = time.time()
    
    while time.time() - start_time < 10:
        msg = master.recv_match(blocking=False)
        if msg:
            msg_type = msg.get_type()
            
            if msg_type == 'EKF_STATUS_REPORT':
                print(f"EKF Health Status:")
                print(f"  Position Variance: {msg.pos_horiz_variance:.6f}")
                print(f"  Velocity Variance: {msg.velocity_variance:.6f}")
                print(f"  Compass Variance: {msg.compass_variance:.6f}")
                
                if (msg.pos_horiz_variance < 1.0 and 
                    msg.velocity_variance < 1.0 and 
                    msg.compass_variance < 1.0):
                    print("✅ EKF status: HEALTHY")
                else:
                    print("⚠️ EKF status: Check calibration")
                break
        
        time.sleep(0.1)
    
    master.close()
