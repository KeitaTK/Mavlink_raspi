#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
EKF Position Estimation Verification Script
モーションキャプチャ + EKF統合の動作確認
"""

from pymavlink import mavutil
import socket
import pickle
import signal
import sys
import time
import math
from datetime import datetime

class EKFVerificationTool:
    def __init__(self, mavlink_port='/dev/ttyACM0', mavlink_baud=921600, udp_port=15769):
        self.mavlink_port = mavlink_port
        self.mavlink_baud = mavlink_baud
        self.udp_port = udp_port
        self.master = None
        self.sock = None
        
        # データ保存用
        self.mocap_position = None
        self.ekf_position = None
        self.ekf_status = {}
        self.position_history = []
        
    def setup_connections(self):
        """MAVLinkとUDP接続のセットアップ"""
        try:
            # MAVLink接続
            print("Setting up MAVLink connection...")
            self.master = mavutil.mavlink_connection(self.mavlink_port, baud=self.mavlink_baud)
            self.master.wait_heartbeat()
            print(f"✓ MAVLink connected to system {self.master.target_system}")
            
            # UDPソケット
            print("Setting up UDP socket...")
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.sock.bind(('', self.udp_port))
            self.sock.setblocking(False)
            print(f"✓ UDP socket listening on port {self.udp_port}")
            
            return True
            
        except Exception as e:
            print(f"✗ Connection setup failed: {e}")
            return False
    
    def verify_ekf_configuration(self):
        """EKF設定状態を確認"""
        print("\n=== EKF Configuration Check ===")
        
        ekf_params = {
            'AHRS_EKF_TYPE': 3,      # 期待値
            'EK3_ENABLE': 1,
            'EK3_SRC1_POSXY': 6,     # ExternalNav
            'EK3_SRC1_POSZ': 6,      # ExternalNav
            'EK3_SRC1_YAW': 6        # ExternalNav
        }
        
        config_ok = True
        
        for param, expected in ekf_params.items():
            try:
                self.master.mav.param_request_read_send(
                    self.master.target_system,
                    self.master.target_component,
                    param.encode('utf-8'),
                    -1
                )
                
                msg = self.master.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
                if msg:
                    value = msg.param_value
                    status = "✓" if value == expected else "✗"
                    print(f"  {status} {param} = {value} (expected: {expected})")
                    
                    if value != expected:
                        config_ok = False
                        
                time.sleep(0.1)
                
            except Exception as e:
                print(f"  ✗ Failed to read {param}: {e}")
                config_ok = False
        
        if config_ok:
            print("✅ EKF configuration is correct")
        else:
            print("⚠️ EKF configuration needs adjustment")
            
        return config_ok
    
    def request_ekf_messages(self):
        """EKFメッセージのリクエスト"""
        try:
            # LOCAL_POSITION_NEDリクエスト
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_GET_MESSAGE_INTERVAL,
                0,
                mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,
                100000,  # 10Hz
                0, 0, 0, 0, 0
            )
            
            # EKF_STATUS_REPORTリクエスト
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_GET_MESSAGE_INTERVAL,
                0,
                mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT,
                1000000,  # 1Hz
                0, 0, 0, 0, 0
            )
            
            print("✓ EKF message requests sent")
            return True
            
        except Exception as e:
            print(f"✗ Failed to request EKF messages: {e}")
            return False
    
    def get_latest_mocap_data(self):
        """最新のモーションキャプチャデータを取得"""
        try:
            newest_data = None
            packets_read = 0
            
            while True:
                try:
                    data_bytes, sender_addr = self.sock.recvfrom(1024)
                    if data_bytes:
                        newest_data = data_bytes
                        packets_read += 1
                except socket.error:
                    break
            
            if newest_data:
                data = pickle.loads(newest_data)
                self.mocap_position = {
                    'pos': data['position'],    # [x, y, z]
                    'quat': data['quaternion'], # [w, x, y, z]
                    'timestamp': time.time()
                }
                return True
                
        except Exception as e:
            print(f"MOCAP data error: {e}")
            
        return False
    
    def monitor_ekf_responses(self):
        """EKF応答を監視"""
        msg = self.master.recv_match(blocking=False)
        if msg:
            msg_type = msg.get_type()
            
            if msg_type == 'LOCAL_POSITION_NED':
                self.ekf_position = {
                    'x': msg.x,
                    'y': msg.y,
                    'z': msg.z,
                    'vx': msg.vx,
                    'vy': msg.vy,
                    'vz': msg.vz,
                    'timestamp': time.time()
                }
                return True
                
            elif msg_type == 'EKF_STATUS_REPORT':
                self.ekf_status = {
                    'velocity_variance': msg.velocity_variance,
                    'pos_horiz_variance': msg.pos_horiz_variance,
                    'pos_vert_variance': msg.pos_vert_variance,
                    'compass_variance': msg.compass_variance,
                    'timestamp': time.time()
                }
                return True
                
        return False
    
    def compare_positions(self):
        """モーションキャプチャとEKF位置の比較"""
        if not self.mocap_position or not self.ekf_position:
            return None
        
        mocap_pos = self.mocap_position['pos']
        ekf_pos = (self.ekf_position['x'], self.ekf_position['y'], self.ekf_position['z'])
        
        # 位置差分計算
        diff_x = abs(mocap_pos[0] - ekf_pos[0])
        diff_y = abs(mocap_pos[1] - ekf_pos[1])
        diff_z = abs(mocap_pos[2] - ekf_pos[2])
        
        # 総合誤差
        total_error = math.sqrt(diff_x**2 + diff_y**2 + diff_z**2)
        
        comparison = {
            'mocap_pos': mocap_pos,
            'ekf_pos': ekf_pos,
            'diff': (diff_x, diff_y, diff_z),
            'total_error': total_error,
            'timestamp': time.time()
        }
        
        # 履歴に追加
        self.position_history.append(comparison)
        
        return comparison
    
    def evaluate_accuracy(self, comparison):
        """精度評価"""
        if not comparison:
            return "No data"
        
        error = comparison['total_error']
        
        if error < 0.05:      # 5cm以内
            return "Excellent"
        elif error < 0.10:    # 10cm以内
            return "Good"
        elif error < 0.20:    # 20cm以内
            return "Acceptable"
        elif error < 0.50:    # 50cm以内
            return "Poor"
        else:
            return "Very Poor"
    
    def display_status(self):
        """現在の状況を表示"""
        print(f"\n{'='*60}")
        print(f"EKF Position Estimation Status - {datetime.now().strftime('%H:%M:%S')}")
        print(f"{'='*60}")
        
        # モーションキャプチャデータ
        if self.mocap_position:
            pos = self.mocap_position['pos']
            age = time.time() - self.mocap_position['timestamp']
            print(f"MOCAP Position: N={pos[0]:+7.3f}m E={pos[1]:+7.3f}m D={pos[2]:+7.3f}m (Age: {age:.1f}s)")
        else:
            print("MOCAP Position: No data")
        
        # EKF位置データ
        if self.ekf_position:
            pos = self.ekf_position
            age = time.time() - pos['timestamp']
            print(f"EKF Position:   N={pos['x']:+7.3f}m E={pos['y']:+7.3f}m D={pos['z']:+7.3f}m (Age: {age:.1f}s)")
            print(f"EKF Velocity:   N={pos['vx']:+6.2f}m/s E={pos['vy']:+6.2f}m/s D={pos['vz']:+6.2f}m/s")
        else:
            print("EKF Position:   No data")
        
        # EKF健全性
        if self.ekf_status:
            ekf = self.ekf_status
            print(f"\nEKF Health:")
            print(f"  Position H: {ekf['pos_horiz_variance']:.6f}")
            print(f"  Position V: {ekf['pos_vert_variance']:.6f}")
            print(f"  Velocity:   {ekf['velocity_variance']:.6f}")
            print(f"  Compass:    {ekf['compass_variance']:.6f}")
            
            # 健全性評価
            pos_healthy = ekf['pos_horiz_variance'] < 1.0 and ekf['pos_vert_variance'] < 1.0
            vel_healthy = ekf['velocity_variance'] < 1.0
            compass_healthy = ekf['compass_variance'] < 1.0
            overall_health = pos_healthy and vel_healthy and compass_healthy
            
            print(f"  Overall: {'✅ GOOD' if overall_health else '⚠️ WARNING'}")
        
        # 位置比較
        comparison = self.compare_positions()
        if comparison:
            diff = comparison['diff']
            error = comparison['total_error']
            accuracy = self.evaluate_accuracy(comparison)
            
            print(f"\nPosition Comparison:")
            print(f"  Difference: N={diff[0]:6.3f}m E={diff[1]:6.3f}m D={diff[2]:6.3f}m")
            print(f"  Total Error: {error*100:.1f}cm")
            print(f"  Accuracy: {accuracy}")
        
        print(f"{'='*60}")
    
    def run_verification_test(self, duration=30):
        """検証テストの実行"""
        print("=== EKF Position Estimation Verification ===")
        
        if not self.setup_connections():
            return False
        
        if not self.verify_ekf_configuration():
            print("⚠️ Continuing with current configuration...")
        
        if not self.request_ekf_messages():
            return False
        
        print(f"\nRunning verification for {duration} seconds...")
        print("Press Ctrl+C to stop early")
        
        start_time = time.time()
        last_display = 0
        
        try:
            while time.time() - start_time < duration:
                # データ取得
                self.get_latest_mocap_data()
                self.monitor_ekf_responses()
                
                # 5秒ごとに状態表示
                if time.time() - last_display >= 5:
                    self.display_status()
                    last_display = time.time()
                
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\nTest stopped by user")
        
        # 最終結果
        self.display_final_results()
        return True
    
    def display_final_results(self):
        """最終結果の表示"""
        print(f"\n{'='*60}")
        print("FINAL VERIFICATION RESULTS")
        print(f"{'='*60}")
        
        if len(self.position_history) > 0:
            errors = [comp['total_error'] for comp in self.position_history]
            avg_error = sum(errors) / len(errors)
            max_error = max(errors)
            min_error = min(errors)
            
            print(f"Position Accuracy Statistics:")
            print(f"  Samples: {len(errors)}")
            print(f"  Average Error: {avg_error*100:.1f}cm")
            print(f"  Maximum Error: {max_error*100:.1f}cm")
            print(f"  Minimum Error: {min_error*100:.1f}cm")
            
            # 精度分布
            excellent = sum(1 for e in errors if e < 0.05)
            good = sum(1 for e in errors if 0.05 <= e < 0.10)
            acceptable = sum(1 for e in errors if 0.10 <= e < 0.20)
            poor = sum(1 for e in errors if e >= 0.20)
            
            print(f"\nAccuracy Distribution:")
            print(f"  Excellent (<5cm):   {excellent:3d} ({excellent/len(errors)*100:.1f}%)")
            print(f"  Good (5-10cm):      {good:3d} ({good/len(errors)*100:.1f}%)")
            print(f"  Acceptable (10-20cm): {acceptable:3d} ({acceptable/len(errors)*100:.1f}%)")
            print(f"  Poor (>20cm):       {poor:3d} ({poor/len(errors)*100:.1f}%)")
            
            # 総合評価
            if avg_error < 0.05:
                overall = "✅ EXCELLENT - EKF estimation is highly accurate"
            elif avg_error < 0.10:
                overall = "✅ GOOD - EKF estimation is working well"
            elif avg_error < 0.20:
                overall = "⚠️ ACCEPTABLE - EKF estimation needs tuning"
            else:
                overall = "❌ POOR - EKF estimation has issues"
            
            print(f"\nOverall Assessment: {overall}")
            
        else:
            print("❌ No position data collected - Check connections and setup")
    
    def cleanup(self):
        """リソースのクリーンアップ"""
        if self.sock:
            self.sock.close()
        if self.master:
            self.master.close()

def signal_handler(signum, frame):
    """Ctrl+C ハンドラー"""
    print("\nShutting down...")
    sys.exit(0)

def main():
    """メイン関数"""
    signal.signal(signal.SIGINT, signal_handler)
    
    # テストツール作成
    verifier = EKFVerificationTool()
    
    try:
        # 検証テスト実行（30秒間）
        verifier.run_verification_test(duration=30)
        
    except Exception as e:
        print(f"Test failed: {e}")
        
    finally:
        verifier.cleanup()

if __name__ == "__main__":
    main()
