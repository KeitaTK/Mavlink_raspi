#!/usr/bin/env python3
"""
GPS診断ツール - ArduPilot GPS接続とメッセージ受信の詳細診断（修正版）
"""

import time
import sys
from pymavlink import mavutil

# 設定
CONNECTION_PORT = '/dev/ttyAMA0'
BAUD_RATE = 115200
DIAGNOSTIC_TIME = 30  # 診断実行時間（秒）

def safe_decode_param_id(param_id):
    """param_idを安全にデコードする関数"""
    if isinstance(param_id, bytes):
        return param_id.decode('utf-8').strip()
    elif isinstance(param_id, str):
        return param_id.strip()
    else:
        return str(param_id).strip()

def connect_to_vehicle(port, baud):
    """機体に接続"""
    print(f"Connecting to vehicle on: {port} at {baud} baud")
    try:
        master = mavutil.mavlink_connection(port, baud=baud, wait_heartbeat=True)
        print(f"✓ Connected! System {master.target_system} Component {master.target_component}")
        return master
    except Exception as e:
        print(f"✗ Connection failed: {e}")
        return None

def request_gps_streams(master):
    """GPSメッセージストリームを明示的に要求"""
    print("\nRequesting GPS data streams...")
    
    # 方法1: REQUEST_DATA_STREAM（古い方法だが互換性が高い）
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_POSITION,  # GPS位置情報
        1,  # 1Hz
        1   # start_stop (1=start)
    )
    
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS,  # 生センサーデータ
        1,  # 1Hz  
        1   # start_stop (1=start)
    )
    
    # 方法2: MESSAGE_INTERVAL（新しい方法）
    # GPS_RAW_INTメッセージを1秒間隔で要求
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_GPS_RAW_INT,  # GPS_RAW_INT
        1000000,  # 1秒間隔（マイクロ秒）
        0, 0, 0, 0, 0
    )
    
    # GLOBAL_POSITION_INTメッセージを1秒間隔で要求
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
        0,
        mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT,  # GLOBAL_POSITION_INT
        1000000,  # 1秒間隔（マイクロ秒）
        0, 0, 0, 0, 0
    )
    
    print("✓ Stream requests sent")

def check_gps_parameters(master):
    """GPS関連パラメータの確認（修正版）"""
    print("\nChecking GPS parameters...")
    
    # 重要なGPSパラメータをリクエスト
    gps_params = ['GPS_TYPE', 'GPS_AUTO_CONFIG', 'GPS_AUTO_SWITCH', 'GPS_BLEND_MASK']
    
    for param_name in gps_params:
        # パラメータリクエスト（bytesで送信）
        master.mav.param_request_read_send(
            master.target_system,
            master.target_component,
            param_name.encode('utf-8'),
            -1
        )
        
        # レスポンス待機
        param_msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
        if param_msg:
            # param_idを安全にデコード
            received_param_name = safe_decode_param_id(param_msg.param_id)
            if received_param_name == param_name:
                print(f"  {param_name}: {param_msg.param_value}")
            else:
                print(f"  {param_name}: Received {received_param_name} instead (value: {param_msg.param_value})")
        else:
            print(f"  {param_name}: No response (timeout)")

def diagnose_gps(master):
    """GPS詳細診断"""
    print(f"\n{'='*50}")
    print("GPS DIAGNOSTIC START")
    print(f"{'='*50}")
    
    # パラメータ確認
    check_gps_parameters(master)
    
    # データストリーム要求
    request_gps_streams(master)
    
    # 短い待機時間をおく
    print("\nWaiting 3 seconds for stream setup...")
    time.sleep(3)
    
    # 診断統計
    stats = {
        'heartbeat_count': 0,
        'gps_raw_count': 0,
        'global_pos_count': 0,
        'gps_status_count': 0,
        'sys_status_count': 0,
        'other_msg_count': 0
    }
    
    print(f"\nMonitoring messages for {DIAGNOSTIC_TIME} seconds...")
    print("Time | Message Type | Content")
    print("-" * 70)
    
    start_time = time.time()
    
    while time.time() - start_time < DIAGNOSTIC_TIME:
        msg = master.recv_match(blocking=False)
        
        if msg is not None:
            msg_type = msg.get_type()
            current_time = time.time() - start_time
            
            if msg_type == 'HEARTBEAT':
                stats['heartbeat_count'] += 1
                if stats['heartbeat_count'] % 5 == 1:  # 5回に1回だけ表示
                    print(f"{current_time:5.1f}s | HEARTBEAT    | System OK")
                    
            elif msg_type == 'GPS_RAW_INT':
                stats['gps_raw_count'] += 1
                fix_types = {0: "No GPS", 1: "No Fix", 2: "2D", 3: "3D", 4: "DGPS", 5: "RTK_Float", 6: "RTK_Fixed"}
                fix_type = fix_types.get(msg.fix_type, f"Unknown({msg.fix_type})")
                hdop = msg.eph / 100.0 if msg.eph != 65535 else "N/A"
                print(f"{current_time:5.1f}s | GPS_RAW_INT  | Fix:{fix_type}, Sats:{msg.satellites_visible}, HDOP:{hdop}")
                
            elif msg_type == 'GLOBAL_POSITION_INT':
                stats['global_pos_count'] += 1
                lat = msg.lat / 1e7
                lon = msg.lon / 1e7
                alt = msg.alt / 1000.0
                print(f"{current_time:5.1f}s | GLOBAL_POS   | Lat:{lat:.6f}, Lon:{lon:.6f}, Alt:{alt:.1f}m")
                
            elif msg_type == 'GPS_STATUS':
                stats['gps_status_count'] += 1
                print(f"{current_time:5.1f}s | GPS_STATUS   | Satellites visible: {msg.satellites_visible}")
                
            elif msg_type == 'SYS_STATUS':
                stats['sys_status_count'] += 1
                # センサーヘルス確認（GPS関連）
                sensors = msg.onboard_control_sensors_health
                gps_healthy = bool(sensors & mavutil.mavlink.MAV_SYS_STATUS_SENSOR_GPS)
                if stats['sys_status_count'] % 10 == 1:  # 10回に1回表示
                    print(f"{current_time:5.1f}s | SYS_STATUS   | GPS Health: {'OK' if gps_healthy else 'FAIL'}")
                    
            else:
                stats['other_msg_count'] += 1
                # 珍しいメッセージは表示
                if msg_type in ['GPS2_RAW', 'GPS2_RTK', 'GPS_RTK', 'GPS_INJECT_DATA']:
                    print(f"{current_time:5.1f}s | {msg_type:12} | Additional GPS data")
                
        time.sleep(0.1)
    
    # 診断結果サマリー
    print(f"\n{'='*50}")
    print("DIAGNOSTIC SUMMARY")
    print(f"{'='*50}")
    
    for key, value in stats.items():
        status = "✓ OK" if value > 0 else "✗ MISSING"
        print(f"{key.replace('_', ' ').title():20}: {value:3d} messages {status}")
    
    # 診断結果と推奨対策
    print(f"\n{'='*50}")
    print("ANALYSIS & RECOMMENDATIONS")
    print(f"{'='*50}")
    
    if stats['gps_raw_count'] == 0 and stats['global_pos_count'] == 0:
        print("🚨 CRITICAL: No GPS messages received at all")
        print("   Possible causes:")
        print("   1. GPS module not connected to Pixhawk")
        print("   2. Wrong GPS port configuration")
        print("   3. GPS_TYPE parameter set to 0 (disabled)")
        print("   4. Faulty GPS module or antenna")
        print("\n   Recommended actions:")
        print("   → Check Mission Planner GPS status")
        print("   → Verify GPS_TYPE parameter (should be 1 for auto)")
        print("   → Check physical GPS connections")
        print("   → Test GPS module LED status")
        
    elif stats['gps_raw_count'] > 0:
        print("✓ GPS module is connected and communicating")
        if stats['global_pos_count'] == 0:
            print("⚠  WARNING: GPS raw data OK, but no position fix")
            print("   → Move to outdoor area with clear sky view")
            print("   → Wait longer for satellite acquisition")
            print("   → Check antenna placement")
        else:
            print("✓ GPS is working correctly!")
            
    else:
        print("❓ Partial GPS data - check connections and parameters")

def main():
    """メイン実行関数"""
    print("ArduPilot GPS Diagnostic Tool (Fixed Version)")
    print("=" * 45)
    
    # 接続
    master = connect_to_vehicle(CONNECTION_PORT, BAUD_RATE)
    if not master:
        print("Cannot proceed without connection. Exiting.")
        sys.exit(1)
    
    # 診断実行
    try:
        diagnose_gps(master)
    except KeyboardInterrupt:
        print("\n\nDiagnostic interrupted by user")
    except Exception as e:
        print(f"\nDiagnostic error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("\nDiagnostic completed.")

if __name__ == "__main__":
    main()
