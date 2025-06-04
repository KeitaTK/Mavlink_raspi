import socket
import pickle
import math
import time
from datetime import datetime
from pymavlink import mavutil

class ArduPilotConnector:
    def __init__(self):
        """ArduPilot接続クラス（添付ファイルの方式を採用）"""
        self.master = None
        self.target_system = None
        self.target_component = None
        self.connect_to_autopilot()
        
    def connect_to_autopilot(self):
        """ArduPilotに接続（添付ファイルの方式）"""
        try:
            print("ArduPilot接続開始...")
            
            # 添付ファイルと同じ接続方式
            print("telem1通信接続")
            self.master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
            
            # heartbeat 受信待ち
            print("Heartbeat待機中...")
            self.master.wait_heartbeat()
            
            # target_system, target_component が設定される
            self.target_system = self.master.target_system
            self.target_component = self.master.target_component
            
            print(f"Heartbeat received from system {self.target_system}, component {self.target_component}")
            
            # 接続確認のためAUTOPILOT_VERSIONを要求
            self.master.mav.autopilot_version_request_send(
                self.target_system,
                self.target_component
            )
            
            # AUTOPILOT_VERSIONメッセージの受信
            msg = self.master.recv_match(type='AUTOPILOT_VERSION', blocking=True, timeout=5)
            if msg:
                print("ArduPilot接続成功!")
                print(f"Flight SW Version: {msg.flight_sw_version}")
                return True
            else:
                print("AUTOPILOT_VERSION受信タイムアウト")
                return False
                
        except Exception as e:
            print(f"ArduPilot接続失敗: {e}")
            self.master = None
            return False
    
    def send_gps_input(self, lat, lon, alt, yaw_deg):
        """GPS_INPUTメッセージ送信"""
        if not self.master:
            return False
        
        try:
            # ヨー角をセンチ度に変換
            yaw_cdeg = int(yaw_deg * 100) if yaw_deg is not None else 0
            
            # GPS_INPUTメッセージ送信
            self.master.mav.gps_input_send(
                int(time.time() * 1e6),  # time_usec
                0,                       # gps_id
                0,                       # ignore_flags
                0,                       # time_week_ms
                0,                       # time_week
                3,                       # fix_type: 3D Fix
                int(lat * 1e7),         # lat (degE7)
                int(lon * 1e7),         # lon (degE7) 
                alt,                    # alt (m)
                0.8,                    # hdop
                1.0,                    # vdop
                0.0, 0.0, 0.0,          # vn, ve, vd (m/s)
                0.1,                    # speed_accuracy
                0.01,                   # horiz_accuracy
                0.01,                   # vert_accuracy
                12,                     # satellites_visible
                yaw_cdeg                # yaw (cdeg)
            )
            return True
            
        except Exception as e:
            print(f"GPS_INPUT送信エラー: {e}")
            return False
    
    def check_connection(self):
        """接続状態確認"""
        if not self.master:
            return False
        
        try:
            # パラメータ要求で接続確認
            self.master.mav.param_request_read_send(
                self.target_system,
                self.target_component,
                b'EK3_SRC1_YAW',
                -1
            )
            
            # 応答確認
            msg = self.master.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
            if msg:
                print(f"接続確認OK: EK3_SRC1_YAW = {msg.param_value}")
                return True
            else:
                print("接続確認タイムアウト")
                return False
                
        except Exception as e:
            print(f"接続確認エラー: {e}")
            return False

class UDPReceiver:
    def __init__(self, host='0.0.0.0', port=15769):
        """UDP受信クラス"""
        self.host = host
        self.port = port
        self.socket = None
        self.setup_udp()
    
    def setup_udp(self):
        """UDP受信設定"""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.socket.bind((self.host, self.port))
            print(f"UDP受信開始: {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"UDP設定エラー: {e}")
            return False
    
    def receive_data(self):
        """データ受信"""
        try:
            data, addr = self.socket.recvfrom(2048)
            return pickle.loads(data), addr
        except Exception as e:
            print(f"UDP受信エラー: {e}")
            return None, None
    
    def close(self):
        """ソケットクローズ"""
        if self.socket:
            self.socket.close()

def motive_to_ardupilot_bridge():
    """
    Motive→ArduPilotブリッジメイン関数
    添付ファイルの接続方式を使用
    """
    
    print("=" * 60)
    print("Motive to ArduPilot Bridge")
    print("=" * 60)
    
    # ArduPilot接続
    ardupilot = ArduPilotConnector()
    if not ardupilot.master:
        print("ERROR: ArduPilot接続に失敗しました")
        return
    
    # 接続確認
    if not ardupilot.check_connection():
        print("WARNING: 接続確認に失敗しましたが続行します")
    
    # UDP受信設定
    udp_receiver = UDPReceiver('0.0.0.0', 15769)
    if not udp_receiver.socket:
        print("ERROR: UDP受信設定に失敗しました")
        return
    
    print("ブリッジ開始 - Motiveデータ受信中...")
    print("Ctrl+Cで停止")
    
    # 統計変数
    packet_count = 0
    success_count = 0
    error_count = 0
    last_stats_time = time.time()
    last_connection_check = time.time()
    
    try:
        while True:
            # 定期的な接続確認（60秒ごと）
            current_time = time.time()
            if current_time - last_connection_check >= 60.0:
                if not ardupilot.check_connection():
                    print("WARNING: ArduPilot接続が不安定です")
                last_connection_check = current_time
            
            # Motiveデータ受信
            received_data, addr = udp_receiver.receive_data()
            
            if received_data is None:
                continue
                
            packet_count += 1
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            
            # データ処理
            status = received_data.get('status', 'UNKNOWN')
            
            if status == 'SUCCESS':
                # GPS情報取得
                lat = received_data.get('latitude', 0.0)
                lon = received_data.get('longitude', 0.0)
                alt = received_data.get('altitude', 0.0)
                yaw = received_data.get('yaw_degrees', 0.0)
                data_no = received_data.get('data_no', 'N/A')
                
                # ArduPilotに送信
                if ardupilot.send_gps_input(lat, lon, alt, yaw):
                    success_count += 1
                    
                    # 詳細ログ（20パケットごと）
                    if packet_count % 20 == 0:
                        print(f"[{timestamp}] #{packet_count}: GPS送信成功")
                        print(f"  位置: lat={lat:.7f}, lon={lon:.7f}, alt={alt:.3f}m")
                        print(f"  姿勢: yaw={yaw:.2f}°")
                        print(f"  データ番号: {data_no}")
                else:
                    error_count += 1
                    print(f"[{timestamp}] #{packet_count}: GPS送信失敗")
                    
            elif status == 'GPS_CONVERSION_FAILED':
                error_count += 1
                ned_pos = received_data.get('raw_ned_position', [0, 0, 0])
                print(f"[{timestamp}] #{packet_count}: GPS変換失敗")
                print(f"  NED位置: N={ned_pos[0]:.3f}, E={ned_pos[1]:.3f}, D={ned_pos[2]:.3f}")
                
            else:
                error_count += 1
                print(f"[{timestamp}] #{packet_count}: 不明なステータス: {status}")
            
            # 統計表示（10秒ごと）
            if current_time - last_stats_time >= 10.0:
                success_rate = (success_count / packet_count * 100) if packet_count > 0 else 0
                packet_rate = packet_count / (current_time - (last_stats_time - 10.0))
                
                print("-" * 40)
                print(f"統計情報 (10秒間隔)")
                print(f"  総パケット数: {packet_count}")
                print(f"  成功送信数: {success_count}")
                print(f"  エラー数: {error_count}")
                print(f"  成功率: {success_rate:.1f}%")
                print(f"  受信レート: {packet_rate:.1f} pkt/s")
                print("-" * 40)
                
                last_stats_time = current_time
                
    except KeyboardInterrupt:
        print("\n" + "=" * 60)
        print("ブリッジ停止")
        print("=" * 60)
        
        # 最終統計
        total_time = time.time() - (last_stats_time - 10.0)
        avg_rate = packet_count / total_time if total_time > 0 else 0
        
        print(f"総パケット数: {packet_count}")
        print(f"成功送信数: {success_count}")
        print(f"エラー数: {error_count}")
        print(f"平均受信レート: {avg_rate:.2f} pkt/s")
        
        if packet_count > 0:
            success_percentage = success_count / packet_count * 100
            print(f"総合成功率: {success_percentage:.2f}%")
        
        print("=" * 60)
        
    except Exception as e:
        print(f"予期しないエラー: {e}")
        
    finally:
        # クリーンアップ
        udp_receiver.close()
        print("リソースをクリーンアップしました")

if __name__ == "__main__":
    motive_to_ardupilot_bridge()
