from pymavlink import mavutil
import socket
import pickle
import signal
import sys
import time

class FrequencyController:
    def __init__(self, target_hz=20):
        self.target_hz = target_hz
        self.interval = 1.0 / target_hz
        self.last_send_time = 0
    
    def should_send(self):
        current_time = time.time()
        if current_time - self.last_send_time >= self.interval:
            self.last_send_time = current_time
            return True
        return False

class EKFMonitor:
    def __init__(self, master):
        self.master = master
        self.ekf_status = {}
        self.local_position = {}
        self.last_position_time = 0
    
    def request_ekf_status(self):
        """EKF状態レポートをリクエスト"""
        try:
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_GET_MESSAGE_INTERVAL,
                0,  # confirmation
                mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT,
                1000000,  # 1Hz
                0, 0, 0, 0, 0
            )
            print("EKF status report requested")
        except Exception as e:
            print(f"EKF request failed: {e}")
    
    def monitor_responses(self):
        """EKF応答とローカル位置を監視"""
        msg = self.master.recv_match(blocking=False)
        if msg:
            msg_type = msg.get_type()
            
            if msg_type == 'LOCAL_POSITION_NED':
                self.local_position = {
                    'x': msg.x,
                    'y': msg.y, 
                    'z': msg.z,
                    'vx': msg.vx,
                    'vy': msg.vy,
                    'vz': msg.vz,
                    'time_boot_ms': msg.time_boot_ms
                }
                self.last_position_time = time.time()
                return True
                
            elif msg_type == 'EKF_STATUS_REPORT':
                self.ekf_status = {
                    'velocity_variance': msg.velocity_variance,
                    'pos_horiz_variance': msg.pos_horiz_variance,
                    'pos_vert_variance': msg.pos_vert_variance,
                    'compass_variance': msg.compass_variance,
                    'terrain_alt_variance': msg.terrain_alt_variance
                }
                return True
                
        return False
    
    def display_status(self):
        """EKF状態を表示"""
        if self.local_position:
            pos = self.local_position
            age = time.time() - self.last_position_time
            
            print(f"\n--- EKF Position Status ---")
            print(f"Local Position: N={pos['x']:+7.3f}m E={pos['y']:+7.3f}m D={pos['z']:+7.3f}m")
            print(f"Local Velocity: N={pos['vx']:+6.2f}m/s E={pos['vy']:+6.2f}m/s D={pos['vz']:+6.2f}m/s")
            print(f"Data Age: {age:.1f}s")
            
        if self.ekf_status:
            ekf = self.ekf_status
            print(f"EKF Variances:")
            print(f"  Position H: {ekf['pos_horiz_variance']:.6f}")
            print(f"  Position V: {ekf['pos_vert_variance']:.6f}")
            print(f"  Velocity:   {ekf['velocity_variance']:.6f}")
            print(f"  Compass:    {ekf['compass_variance']:.6f}")
            
            # 健全性評価
            pos_healthy = ekf['pos_horiz_variance'] < 1.0 and ekf['pos_vert_variance'] < 1.0
            vel_healthy = ekf['velocity_variance'] < 1.0
            compass_healthy = ekf['compass_variance'] < 1.0
            
            overall_health = pos_healthy and vel_healthy and compass_healthy
            print(f"Overall EKF Health: {'✅ GOOD' if overall_health else '⚠️ WARNING'}")
            print("-" * 40)

def get_latest_udp_data(sock):
    """UDPソケットから最新データのみを取得"""
    sock.setblocking(False)
    newest_data = None
    packets_read = 0
    
    try:
        while True:
            data_bytes, sender_addr = sock.recvfrom(1024)
            if data_bytes:
                newest_data = data_bytes
                packets_read += 1
    except socket.error:
        pass  # データなしでループ終了
    
    return newest_data, packets_read

def signal_handler(signum, frame):
    """Ctrl+C ハンドラー"""
    print("\nCtrl+C pressed! Closing socket and exiting...")
    if 'sock' in globals():
        sock.close()
        print("Socket closed")
    if 'master' in globals() and master:
        master.close()
        print("MAVLink connection closed")
    sys.exit(0)

def integrated_mocap_ekf_test():
    """モーションキャプチャ + EKF統合テスト"""
    global sock, master
    
    # SIGINT (Ctrl+C) ハンドラー設定
    signal.signal(signal.SIGINT, signal_handler)
    
    # UDP + MAVLink設定
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('', 15769))
    
    try:
        master = mavutil.mavlink_connection('/dev/ttyACM0', baud=921600)
        master.wait_heartbeat()
        print("✓ MAVLink connection established")
        print(f"  System ID: {master.target_system}")
        print(f"  Component ID: {master.target_component}")
    except Exception as e:
        print(f"✗ MAVLink connection failed: {e}")
        return
    
    # EKF監視セットアップ
    ekf_monitor = EKFMonitor(master)
    ekf_monitor.request_ekf_status()
    
    # 送信制御
    freq_controller = FrequencyController(target_hz=20)
    
    print("Testing MOCAP → EKF integration...")
    print("Press Ctrl+C to stop")
    print("=" * 60)
    
    packet_count = 0
    mavlink_send_count = 0
    status_display_counter = 0
    
    try:
        while True:
            # UDPデータ受信・送信
            newest_data, packets_read = get_latest_udp_data(sock)
            
            if newest_data:
                packet_count += 1
                
                if freq_controller.should_send():
                    try:
                        data = pickle.loads(newest_data)
                        pos = data['position']    # [x, y, z]
                        quat = data['quaternion'] # [w, x, y, z]
                        
                        # データ検証
                        if len(pos) == 3 and len(quat) == 4:
                            # MAVLink送信
                            time_usec = int(time.time() * 1000000)
                            master.mav.att_pos_mocap_send(
                                time_usec,      # time_usec
                                quat,           # quaternion array
                                pos[0],         # x (North)
                                pos[1],         # y (East)
                                pos[2]          # z (Down)
                            )
                            mavlink_send_count += 1
                            
                            # 送信確認表示（10回に1回）
                            if mavlink_send_count % 10 == 1:
                                print(f"[{mavlink_send_count:03d}] MAVLink → "
                                      f"Pos: ({pos[0]:+6.2f}, {pos[1]:+6.2f}, {pos[2]:+6.2f}) | "
                                      f"RX: {packet_count} pkts")
                        
                    except Exception as e:
                        print(f"Data processing error: {e}")
            
            # EKF状態監視
            ekf_monitor.monitor_responses()
            
            # 状態表示（約5秒ごと）
            status_display_counter += 1
            if status_display_counter >= 5000:  # 約5秒（0.001秒 × 5000）
                ekf_monitor.display_status()
                status_display_counter = 0
            
            time.sleep(0.001)
            
    except KeyboardInterrupt:
        print("\nTest completed")
    except Exception as e:
        print(f"Main loop error: {e}")
    finally:
        sock.close()
        if master:
            master.close()
        print(f"\nStatistics:")
        print(f"  Total UDP packets received: {packet_count}")
        print(f"  Total MAVLink messages sent: {mavlink_send_count}")

if __name__ == "__main__":
    integrated_mocap_ekf_test()
