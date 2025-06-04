import socket
import pickle
import time
from datetime import datetime

def simple_data_receiver():
    """
    Motiveからのデータを受信して表示するだけのシンプルな受信機
    """
    # UDP設定
    HOST = '0.0.0.0'  # 全てのインターフェースで受信
    PORT = 15769      # 受信ポート
    
    # UDPソケット作成
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind((HOST, PORT))
    
    print("=" * 60)
    print("🎯 Motive Data Receiver Started")
    print(f"📡 Listening on {HOST}:{PORT}")
    print("=" * 60)
    
    packet_count = 0
    
    try:
        while True:
            # データ受信
            data, addr = sock.recvfrom(4096)
            packet_count += 1
            
            # 受信時刻
            timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
            
            try:
                # データをデシリアライズ
                received_data = pickle.loads(data)
                
                # データ表示
                print(f"\n[{timestamp}] 📦 Packet #{packet_count} from {addr[0]}")
                print("-" * 50)
                
                # ステータスチェック
                status = received_data.get('status', 'NO_STATUS')
                
                if status == 'SUCCESS':
                    # 成功データの表示
                    print("✅ STATUS: SUCCESS")
                    print(f"🆔 ID: {received_data.get('id', 'N/A')}")
                    print(f"📍 Latitude:  {received_data.get('latitude', 0)}")
                    print(f"📍 Longitude: {received_data.get('longitude', 0)}")
                    print(f"📍 Altitude:  {received_data.get('altitude', 0)}")
                    
                    quat = received_data.get('quaternion', [0, 0, 0, 0])
                    print(f"🧭 Quaternion: w={quat[0]:.6f}, x={quat[1]:.6f}, y={quat[2]:.6f}, z={quat[3]:.6f}")
                    
                    print(f"📊 Data No: {received_data.get('data_no', 'N/A')}")
                    print(f"⏰ Data Time: {received_data.get('data_time', 0):.6f}s")
                    
                elif status == 'GPS_CONVERSION_FAILED':
                    # エラーデータの表示
                    print("❌ STATUS: GPS_CONVERSION_FAILED")
                    print(f"🆔 ID: {received_data.get('id', 'N/A')}")
                    print(f"🚨 Error: {received_data.get('error_message', 'Unknown error')}")
                    
                    ned_pos = received_data.get('raw_ned_position', [0, 0, 0])
                    print(f"📍 NED Position: N={ned_pos[0]:.3f}, E={ned_pos[1]:.3f}, D={ned_pos[2]:.3f}")
                    
                    print(f"📊 Data No: {received_data.get('data_no', 'N/A')}")
                    print(f"⏰ Data Time: {received_data.get('data_time', 0):.6f}s")
                    
                else:
                    # 不明なデータの表示
                    print(f"❓ STATUS: {status}")
                    print("📋 Raw Data:")
                    for key, value in received_data.items():
                        print(f"   {key}: {value}")
                
                print(f"📡 Packet Size: {len(data)} bytes")
                
            except Exception as e:
                print(f"❌ Data decode error: {e}")
                print(f"📋 Raw bytes: {data[:100]}..." if len(data) > 100 else f"📋 Raw bytes: {data}")
                
    except KeyboardInterrupt:
        print(f"\n\n🛑 Receiver stopped by user")
        print(f"📊 Total packets received: {packet_count}")
        
    except Exception as e:
        print(f"❌ Receiver error: {e}")
        
    finally:
        sock.close()
        print("🔌 Socket closed")

if __name__ == "__main__":
    simple_data_receiver()
