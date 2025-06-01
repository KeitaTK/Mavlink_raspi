#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
EKF Verification Script - Fixed Parameter Reading
パラメータ読み取りエラーを修正したバージョン
"""

from pymavlink import mavutil
import socket
import pickle
import signal
import sys
import time
import math
from datetime import datetime

class EKFVerificationToolFixed:
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
    
    def read_parameter_safe(self, param_name, max_retries=3):
        """安全なパラメータ読み取り（検索結果の修正を適用）"""
        for attempt in range(max_retries):
            try:
                # 検索結果の文字列エンコード確認
                encoded_name = param_name.encode('utf-8')
                print(f"  Reading {param_name} (attempt {attempt + 1}/{max_retries})")
                
                # パラメータリクエスト送信
                self.master.mav.param_request_read_send(
                    self.master.target_system,
                    self.master.target_component,
                    encoded_name,
                    -1
                )
                
                # 応答待ち（タイムアウト延長）
                start_time = time.time()
                while time.time() - start_time < 5.0:  # 5秒タイムアウト
                    msg = self.master.recv_match(type='PARAM_VALUE', blocking=False)
                    if msg:
                        # パラメータ名の比較（デコード処理改善）
                        received_name = msg.param_id.decode('utf-8').rstrip('\x00')
                        if received_name == param_name:
                            print(f"    ✓ {param_name} = {msg.param_value}")
                            return msg.param_value
                    time.sleep(0.1)
                
                print(f"    Timeout on attempt {attempt + 1}")
                time.sleep(0.5)  # リトライ前の待機
                
            except Exception as e:
                print(f"    Error on attempt {attempt + 1}: {e}")
                time.sleep(0.5)
        
        print(f"    ✗ Failed to read {param_name} after {max_retries} attempts")
        return None
    
    def verify_ekf_configuration_fixed(self):
        """修正されたEKF設定確認"""
        print("\n=== EKF Configuration Check (Fixed) ===")
        
        ekf_params = {
            'AHRS_EKF_TYPE': 3,
            'EK3_ENABLE': 1,
            'EK3_SRC1_POSXY': 6,
            'EK3_SRC1_POSZ': 6,
            'EK3_SRC1_YAW': 6
        }
        
        config_ok = True
        actual_values = {}
        
        for param_name, expected in ekf_params.items():
            value = self.read_parameter_safe(param_name)
            actual_values[param_name] = value
            
            if value is not None:
                if abs(value - expected) < 0.1:  # 浮動小数点誤差を考慮
                    print(f"  ✓ {param_name} = {value} ✓")
                else:
                    print(f"  ✗ {param_name} = {value} (expected: {expected})")
                    config_ok = False
            else:
                print(f"  ✗ {param_name} = FAILED TO READ")
                config_ok = False
        
        print(f"\nConfiguration Summary:")
        for param, value in actual_values.items():
            if value is not None:
                print(f"  {param}: {value}")
        
        if config_ok:
            print("✅ EKF configuration is correct")
        else:
            print("⚠️ EKF configuration has issues")
            print("\nTo fix configuration, run:")
            print("python3 setup_ekf_params.py")
            
        return config_ok
    
    def request_ekf_messages_enhanced(self):
        """強化されたEKFメッセージリクエスト"""
        print("\nRequesting EKF messages...")
        
        messages_to_request = [
            (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 200000, "LOCAL_POSITION_NED"),
            (mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 1000000, "EKF_STATUS_REPORT"),
            (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 500000, "GLOBAL_POSITION_INT")
        ]
        
        success_count = 0
        
        for msg_id, interval, name in messages_to_request:
            try:
                self.master.mav.command_long_send(
                    self.master.target_system,
                    self.master.target_component,
                    mavutil.mavlink.MAV_CMD_GET_MESSAGE_INTERVAL,
                    0,
                    msg_id,
                    interval,
                    0, 0, 0, 0, 0
                )
                print(f"  ✓ Requested {name}")
                success_count += 1
                time.sleep(0.2)
                
            except Exception as e:
                print(f"  ✗ Failed to request {name}: {e}")
        
        print(f"Successfully requested {success_count}/{len(messages_to_request)} messages")
        return success_count > 0
    
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
                    'pos': data['position'],
                    'quat': data['quaternion'],
                    'timestamp': time.time()
                }
                return True
                
        except Exception as e:
            print(f"MOCAP data error: {e}")
            
        return False
    
    def monitor_ekf_responses_enhanced(self):
        """強化されたEKF応答監視"""
        responses = []
        
        # より多くのメッセージを同時に確認
        for _ in range(10):  # 複数回チェック
            msg = self.master.recv_match(blocking=False)
            if msg:
                msg_type = msg.get_type()
                responses.append(msg_type)
                
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
                    
                elif msg_type == 'EKF_STATUS_REPORT':
                    self.ekf_status = {
                        'velocity_variance': msg.velocity_variance,
                        'pos_horiz_variance': msg.pos_horiz_variance,
                        'pos_vert_variance': msg.pos_vert_variance,
                        'compass_variance': msg.compass_variance,
                        'timestamp': time.time()
                    }
        
        # 受信したメッセージタイプを報告
        if responses:
            unique_responses = list(set(responses))
            print(f"    Received: {', '.join(unique_responses)}")
        
        return len(responses) > 0
    
    def debug_mavlink_connection(self):
        """MAVLink接続のデバッグ"""
        print("\n=== MAVLink Connection Debug ===")
        
        try:
            # ハートビート確認
            print("Checking heartbeat...")
            msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=5)
            if msg:
                print(f"  ✓ Heartbeat received from system {msg.get_srcSystem()}")
                print(f"    Type: {msg.type}, Autopilot: {msg.autopilot}")
                print(f"    Base mode: {msg.base_mode}, Custom mode: {msg.custom_mode}")
            else:
                print("  ✗ No heartbeat received")
                return False
            
            # システム状態確認
            print("\nRequesting system status...")
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_GET_MESSAGE_INTERVAL,
                0,
                mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS,
                1000000,  # 1Hz
                0, 0, 0, 0, 0
            )
            
            # 数秒待って応答を確認
            print("Waiting for system messages...")
            messages_received = []
            start_time = time.time()
            
            while time.time() - start_time < 5:
                msg = self.master.recv_match(blocking=False)
                if msg:
                    msg_type = msg.get_type()
                    if msg_type not in messages_received:
                        messages_received.append(msg_type)
                        print(f"  Received: {msg_type}")
                time.sleep(0.1)
            
            print(f"Total message types received: {len(messages_received)}")
            return len(messages_received) > 0
            
        except Exception as e:
            print(f"Debug failed: {e}")
            return False
    
    def display_enhanced_status(self):
        """強化された状態表示"""
        print(f"\n{'='*70}")
        print(f"EKF Position Estimation Status - {datetime.now().strftime('%H:%M:%S')}")
        print(f"{'='*70}")
        
        # モーションキャプチャデータ
        if self.mocap_position:
            pos = self.mocap_position['pos']
            age = time.time() - self.mocap_position['timestamp']
            print(f"MOCAP Position: N={pos[0]:+7.3f}m E={pos[1]:+7.3f}m D={pos[2]:+7.3f}m (Age: {age:.1f}s)")
        else:
            print("MOCAP Position: ❌ No data")
        
        # EKF位置データ
        if self.ekf_position:
            pos = self.ekf_position
            age = time.time() - pos['timestamp']
            print(f"EKF Position:   N={pos['x']:+7.3f}m E={pos['y']:+7.3f}m D={pos['z']:+7.3f}m (Age: {age:.1f}s)")
            print(f"EKF Velocity:   N={pos['vx']:+6.2f}m/s E={pos['vy']:+6.2f}m/s D={pos['vz']:+6.2f}m/s")
        else:
            print("EKF Position:   ❌ No data")
        
        # EKF健全性
        if self.ekf_status:
            ekf = self.ekf_status
            age = time.time() - ekf['timestamp']
            print(f"\nEKF Health (Age: {age:.1f}s):")
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
        else:
            print("\nEKF Health: ❌ No data")
        
        print(f"{'='*70}")
    
    def run_verification_test_fixed(self, duration=30):
        """修正された検証テストの実行"""
        print("=== EKF Position Estimation Verification (Fixed) ===")
        
        if not self.setup_connections():
            return False
        
        # MAVLink接続のデバッグ
        if not self.debug_mavlink_connection():
            print("⚠️ MAVLink connection issues detected")
        
        # EKF設定確認（修正版）
        config_ok = self.verify_ekf_configuration_fixed()
        
        # EKFメッセージリクエスト（強化版）
        if not self.request_ekf_messages_enhanced():
            print("⚠️ Failed to request EKF messages")
        
        print(f"\nRunning verification for {duration} seconds...")
        print("Press Ctrl+C to stop early")
        
        start_time = time.time()
        last_display = 0
        
        try:
            while time.time() - start_time < duration:
                # データ取得
                self.get_latest_mocap_data()
                self.monitor_ekf_responses_enhanced()
                
                # 3秒ごとに状態表示
                if time.time() - last_display >= 3:
                    self.display_enhanced_status()
                    last_display = time.time()
                
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\nTest stopped by user")
        
        return True
    
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
    
    # 修正版テストツール作成
    verifier = EKFVerificationToolFixed()
    
    try:
        # 検証テスト実行
        verifier.run_verification_test_fixed(duration=30)
        
    except Exception as e:
        print(f"Test failed: {e}")
        
    finally:
        verifier.cleanup()

if __name__ == "__main__":
    main()
