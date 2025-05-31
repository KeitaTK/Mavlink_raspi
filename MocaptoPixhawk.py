from pymavlink import mavutil
import socket
import pickle
import signal
import sys
import time

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

def simple_receiver_with_mavlink():
    global sock, master
    
    # SIGINT (Ctrl+C) ハンドラー設定
    signal.signal(signal.SIGINT, signal_handler)
    
    # UDPソケット作成
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('', 15769))
    
    # MAVLink接続設定
    print("Setting up MAVLink connection...")
    try:
        master = mavutil.mavlink_connection('/dev/ttyACM0', baud=921600)
        master.wait_heartbeat()
        print("✓ MAVLink connection established")
        print(f"  System ID: {master.target_system}")
        print(f"  Component ID: {master.target_component}")
    except Exception as e:
        print(f"✗ MAVLink connection failed: {e}")
        print("Continuing with UDP reception only...")
        master = None
    
    print("Waiting for Motive data on port 15769...")
    print("Press Ctrl+C to stop")
    print("-" * 60)
    
    packet_count = 0
    mavlink_send_count = 0
    
    try:
        while True:
            try:
                # タイムアウト付き受信
                sock.settimeout(1.0)
                data_bytes, sender_addr = sock.recvfrom(1024)
                packet_count += 1
                
                # データ処理
                data = pickle.loads(data_bytes)
                pos = data['position']    # [x, y, z]
                quat = data['quaternion'] # [w, x, y, z]
                
                # データ検証
                if pos is None or quat is None:
                    print(f"#{packet_count:04d} | Invalid data: pos={pos}, quat={quat}")
                    continue
                
                if len(pos) != 3 or len(quat) != 4:
                    print(f"#{packet_count:04d} | Wrong data length: pos={len(pos)}, quat={len(quat)}")
                    continue
                
                # コンソール表示
                print(f"#{packet_count:04d} | "
                      f"Pos: ({pos[0]:+6.2f}, {pos[1]:+6.2f}, {pos[2]:+6.2f}) | "
                      f"Quat: ({quat[0]:+5.2f}, {quat[1]:+5.2f}, {quat[2]:+5.2f}, {quat[3]:+5.2f})")
                
                # MAVLink送信（接続が有効な場合のみ）
                if master is not None:
                    try:
                        # タイムスタンプ（マイクロ秒）
                        time_usec = int(time.time() * 1000000)
                        
                        # ATT_POS_MOCAPメッセージ送信（定義に基づく正しい形式）
                        master.mav.att_pos_mocap_send(
                            time_usec,      # uint64_t time_usec
                            quat,           # float[4] q - 配列として渡す (w,x,y,z)
                            pos[0],         # float x (NED North)
                            pos[1],         # float y (NED East)  
                            pos[2]          # float z (NED Down)
                            # covariance は省略（オプション）
                        )
                        
                        mavlink_send_count += 1
                        if mavlink_send_count % 10 == 1:  # 10回に1回表示
                            print(f"      → ATT_POS_MOCAP sent #{mavlink_send_count}")
                        
                    except Exception as e:
                        print(f"      ✗ MAVLink error: {e}")
                        # デバッグ情報
                        print(f"        time_usec: {time_usec} (type: {type(time_usec)})")
                        print(f"        quat: {quat} (type: {type(quat)}, len: {len(quat)})")
                        print(f"        pos: {pos} (type: {type(pos)}, len: {len(pos)})")
                
            except socket.timeout:
                continue  # タイムアウト時は継続
            except KeyError as e:
                print(f"#{packet_count:04d} | Missing key: {e}")
            except Exception as e:
                print(f"#{packet_count:04d} | Data error: {e}")
                
    except Exception as e:
        print(f"Error: {e}")
    finally:
        sock.close()
        if master:
            master.close()
        print(f"\nSummary:")
        print(f"  UDP packets received: {packet_count}")
        print(f"  MAVLink messages sent: {mavlink_send_count}")

if __name__ == "__main__":
    simple_receiver_with_mavlink()
