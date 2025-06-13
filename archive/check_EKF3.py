#!/usr/bin/env python3
"""
EKFとセンサー状態の詳細調査（修正版）
"""

from pymavlink import mavutil
import time

def comprehensive_ekf_diagnosis():
    """包括的なEKF診断"""
    
    # USB接続で確認（より確実）
    master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
    master.wait_heartbeat()
    
    print("=== Comprehensive EKF Diagnosis ===")
    print(f"System ID: {master.target_system}")
    print(f"Component ID: {master.target_component}")
    
    # 1. 重要なEKFパラメータ確認
    print("\n1. EKF Parameters Check:")
    critical_params = {
        'AHRS_EKF_TYPE': 3,
        'EK3_ENABLE': 1,
        'EK3_SRC1_POSXY': 6,
        'EK3_SRC1_POSZ': 6,
        'EK3_SRC1_YAW': 6,
        'EK3_IMU_MASK': 1,
        'EK3_GPS_TYPE': 0,  # GPS無効
        'EK3_ALT_SOURCE': 0
    }
    
    param_results = {}
    for param, expected in critical_params.items():
        try:
            master.mav.param_request_read_send(
                master.target_system,
                master.target_component,
                param.encode('utf-8'),
                -1
            )
            
            msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=5)
            if msg:
                param_name = str(msg.param_id).rstrip('\x00')
                actual = msg.param_value
                status = "✅" if actual == expected else "❌"
                param_results[param] = actual
                print(f"  {status} {param_name} = {actual} (expected: {expected})")
            else:
                param_results[param] = None
                print(f"  ❌ {param} = TIMEOUT")
            
            time.sleep(0.3)
            
        except Exception as e:
            print(f"  ❌ {param} = ERROR: {e}")
    
    # 2. センサー状態詳細確認
    print("\n2. System Sensors Status:")
    try:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_GET_MESSAGE_INTERVAL,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,
            500000,  # 2Hz
            0, 0, 0, 0, 0
        )
        
        start_time = time.time()
        sensor_found = False
        
        while time.time() - start_time < 5:
            msg = master.recv_match(blocking=False)
            if msg and msg.get_type() == 'SYS_STATUS':
                sensor_found = True
                print(f"  Sensors Present: 0x{msg.onboard_control_sensors_present:08X}")
                print(f"  Sensors Enabled: 0x{msg.onboard_control_sensors_enabled:08X}")
                print(f"  Sensors Health:  0x{msg.onboard_control_sensors_health:08X}")
                
                # ビット解析
                sensors = {
                    0x01: "3D Gyro",
                    0x02: "3D Accel", 
                    0x04: "3D Mag",
                    0x08: "Abs Pressure",
                    0x20: "GPS",
                    0x80: "Vision Position",
                    0x200: "External Ground Truth"
                }
                
                print(f"  Sensor Details:")
                for bit, name in sensors.items():
                    present = bool(msg.onboard_control_sensors_present & bit)
                    enabled = bool(msg.onboard_control_sensors_enabled & bit)
                    healthy = bool(msg.onboard_control_sensors_health & bit)
                    
                    if present:
                        status = "✅" if healthy else "❌"
                        en_status = "ON" if enabled else "OFF"
                        print(f"    {status} {name}: {en_status}")
                break
            time.sleep(0.1)
        
        if not sensor_found:
            print(f"  ❌ No SYS_STATUS message received")
    
    except Exception as e:
        print(f"  ❌ Sensor check failed: {e}")
    
    # 3. AHRS状態確認
    print("\n3. AHRS Status:")
    try:
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_GET_MESSAGE_INTERVAL,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
            200000,  # 5Hz
            0, 0, 0, 0, 0
        )
        
        start_time = time.time()
        attitude_found = False
        
        while time.time() - start_time < 3:
            msg = master.recv_match(blocking=False)
            if msg and msg.get_type() == 'ATTITUDE':
                attitude_found = True
                print(f"  ✅ Attitude available: roll={msg.roll:.3f} pitch={msg.pitch:.3f} yaw={msg.yaw:.3f}")
                break
            time.sleep(0.1)
        
        if not attitude_found:
            print(f"  ❌ No ATTITUDE message received")
    
    except Exception as e:
        print(f"  ❌ AHRS check failed: {e}")
    
    # 4. 診断結果
    print(f"\n4. Diagnosis Results:")
    
    # パラメータ問題チェック
    param_issues = []
    for param, expected in critical_params.items():
        if param in param_results:
            if param_results[param] != expected:
                param_issues.append(f"{param}: {param_results[param]} != {expected}")
    
    if param_issues:
        print(f"  ❌ Parameter Issues:")
        for issue in param_issues:
            print(f"    - {issue}")
    else:
        print(f"  ✅ All critical parameters correct")
    
    # 推奨次ステップ
    print(f"\n5. Recommended Next Steps:")
    if param_issues:
        print(f"  1. Fix parameter issues and reboot")
    else:
        print(f"  1. Parameters OK - issue may be in EKF initialization")
        print(f"  2. Try complete Pixhawk reboot")
        print(f"  3. Check if IMU calibration is needed")
        print(f"  4. Verify GPS origin is properly set")
    
    master.close()
    return param_results

if __name__ == "__main__":
    comprehensive_ekf_diagnosis()

