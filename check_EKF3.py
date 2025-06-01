#!/usr/bin/env python3
"""
USB経由でのEKF状態・位置推定監視
（TELEM1でのデータ送信と並行動作）
"""

from pymavlink import mavutil
import time
from datetime import datetime

class EKFMonitorUSB:
    def __init__(self):
        self.master = None
        self.ekf_status = {}
        self.local_position = {}
        self.global_position = {}
        
    def connect_usb(self):
        """USB経由でPixhawk接続"""
        try:
            print("Connecting via USB for EKF monitoring...")
            self.master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
            self.master.wait_heartbeat()
            
            print(f"✅ USB connection established")
            print(f"   System ID: {self.master.target_system}")
            print(f"   Component ID: {self.master.target_component}")
            return True
            
        except Exception as e:
            print(f"❌ USB connection failed: {e}")
            return False
    
    def request_messages(self):
        """EKF関連メッセージをリクエスト"""
        
        message_requests = [
            (mavutil.mavlink.MAVLINK_MSG_ID_EKF_STATUS_REPORT, 1000000, "EKF_STATUS_REPORT"),
            (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 200000, "LOCAL_POSITION_NED"),
            (mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 500000, "GLOBAL_POSITION_INT"),
            (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE, 100000, "ATTITUDE"),
            (mavutil.mavlink.MAVLINK_MSG_ID_SYS_STATUS, 1000000, "SYS_STATUS")
        ]
        
        print("Requesting EKF monitoring messages...")
        for msg_id, interval, name in message_requests:
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
                print(f"  ✅ {name} requested")
                time.sleep(0.1)
            except Exception as e:
                print(f"  ❌ Failed to request {name}: {e}")
    
    def monitor_ekf_status(self):
        """EKF状態を継続監視"""
        
        print("\n=== EKF Status Monitor (USB) ===")
        print("TELEM1: Motion capture data transmission")
        print("USB: EKF status monitoring")
        print("Press Ctrl+C to stop")
        print("-" * 60)
        
        last_display_time = 0
        
        try:
            while True:
                msg = self.master.recv_match(blocking=False)
                if msg:
                    self.process_message(msg)
                
                # 2秒ごとに状態表示
                current_time = time.time()
                if current_time - last_display_time >= 2:
                    self.display_status()
                    last_display_time = current_time
                
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\nMonitoring stopped")
    
    def process_message(self, msg):
        """受信メッセージを処理"""
        
        msg_type = msg.get_type()
        
        if msg_type == 'EKF_STATUS_REPORT':
            self.ekf_status = {
                'velocity_variance': msg.velocity_variance,
                'pos_horiz_variance': msg.pos_horiz_variance,
                'pos_vert_variance': msg.pos_vert_variance,
                'compass_variance': msg.compass_variance,
                'timestamp': time.time()
            }
            
        elif msg_type == 'LOCAL_POSITION_NED':
            self.local_position = {
                'x': msg.x,
                'y': msg.y,
                'z': msg.z,
                'vx': msg.vx,
                'vy': msg.vy,
                'vz': msg.vz,
                'timestamp': time.time()
            }
            
        elif msg_type == 'GLOBAL_POSITION_INT':
            self.global_position = {
                'lat': msg.lat / 10000000.0,
                'lon': msg.lon / 10000000.0,
                'alt': msg.alt / 1000.0,
                'relative_alt': msg.relative_alt / 1000.0,
                'timestamp': time.time()
            }
    
    def display_status(self):
        """現在の状態を表示"""
        
        print(f"\n{'='*60}")
        print(f"EKF Status Monitor - {datetime.now().strftime('%H:%M:%S')}")
        print(f"{'='*60}")
        
        # EKF健全性
        if self.ekf_status:
            ekf = self.ekf_status
            age = time.time() - ekf['timestamp']
            
            print(f"EKF Health (Age: {age:.1f}s):")
            print(f"  Position H: {ekf['pos_horiz_variance']:.6f}")
            print(f"  Position V: {ekf['pos_vert_variance']:.6f}")
            print(f"  Velocity:   {ekf['velocity_variance']:.6f}")
            print(f"  Compass:    {ekf['compass_variance']:.6f}")
            
            # 健全性評価
            pos_healthy = ekf['pos_horiz_variance'] < 1.0 and ekf['pos_vert_variance'] < 1.0
            vel_healthy = ekf['velocity_variance'] < 1.0
            compass_healthy = ekf['compass_variance'] < 1.0
            overall_health = pos_healthy and vel_healthy and compass_healthy
            
            status = "✅ HEALTHY" if overall_health else "⚠️ WARNING"
            print(f"  Overall: {status}")
        else:
            print("EKF Health: No data")
        
        # ローカル位置（EKF推定値）
        if self.local_position:
            pos = self.local_position
            age = time.time() - pos['timestamp']
            
            print(f"\nEKF Position Estimate (Age: {age:.1f}s):")
            print(f"  Local NED: N={pos['x']:+7.3f}m E={pos['y']:+7.3f}m D={pos['z']:+7.3f}m")
            print(f"  Velocity:  N={pos['vx']:+6.2f}m/s E={pos['vy']:+6.2f}m/s D={pos['vz']:+6.2f}m/s")
        else:
            print("\nEKF Position: No data")
        
        # グローバル位置
        if self.global_position:
            gpos = self.global_position
            age = time.time() - gpos['timestamp']
            
            print(f"\nGlobal Position (Age: {age:.1f}s):")
            print(f"  GPS: {gpos['lat']:.7f}, {gpos['lon']:.7f}")
            print(f"  Alt: {gpos['alt']:.1f}m (Rel: {gpos['relative_alt']:.1f}m)")
        
        print(f"{'='*60}")
    
    def run(self):
        """監視実行"""
        if not self.connect_usb():
            return False
        
        self.request_messages()
        time.sleep(2)  # メッセージリクエスト反映待ち
        
        self.monitor_ekf_status()
        
        if self.master:
            self.master.close()
        
        return True

def main():
    """メイン関数"""
    monitor = EKFMonitorUSB()
    monitor.run()

if __name__ == "__main__":
    main()
