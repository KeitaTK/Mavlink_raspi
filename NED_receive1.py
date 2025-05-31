import socket
import pickle
import signal
import sys

def signal_handler(signum, frame):
    """Ctrl+C ハンドラー"""
    print("\nCtrl+C pressed! Closing socket and exiting...")
    if 'sock' in globals():
        sock.close()
        print("Socket closed")
    sys.exit(0)

def simple_receiver_with_ctrl_c():
    global sock
    
    # SIGINT (Ctrl+C) ハンドラー設定
    signal.signal(signal.SIGINT, signal_handler)
    
    # UDPソケット作成
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(('', 15769))
    
    print("Waiting for Motive data on port 15769...")
    print("Press Ctrl+C to stop")
    
    packet_count = 0
    
    try:
        while True:
            try:
                # タイムアウト付き受信
                sock.settimeout(1.0)
                data_bytes, sender_addr = sock.recvfrom(1024)
                packet_count += 1
                
                # データ処理
                data = pickle.loads(data_bytes)
                
                # データ形式: [(id, (x, y, z), (qw, qx, qy, qz), data_no, timestamp)]
                if isinstance(data, list) and len(data) > 0:
                    # リストの最初の要素（タプル）を取得
                    first_entry = data[0]
                    
                    # タプルから各要素を取得
                    rigid_id = first_entry[0]      # ID
                    pos = first_entry[1]           # (x, y, z)
                    quat = first_entry[2]          # (qw, qx, qy, qz)
                    data_no = first_entry[3]       # データ番号
                    timestamp = first_entry[4]     # タイムスタンプ
                    
                    print(f"#{packet_count:04d} | ID: {rigid_id} | "
                          f"Pos: ({pos[0]:+7.3f}, {pos[1]:+7.3f}, {pos[2]:+7.3f}) | "
                          f"Quat: ({quat[0]:+6.3f}, {quat[1]:+6.3f}, {quat[2]:+6.3f}, {quat[3]:+6.3f}) | "
                          f"DataNo: {data_no}")
                          
                else:
                    print(f"#{packet_count:04d} | Unexpected data format: {type(data)}")
                
            except socket.timeout:
                continue  # タイムアウト時は継続
            except IndexError as e:
                print(f"#{packet_count:04d} | Index error: {e}")
            except Exception as e:
                print(f"#{packet_count:04d} | Data error: {e}")
                
    except Exception as e:
        print(f"Error: {e}")
    finally:
        sock.close()
        print(f"Total packets: {packet_count}")

if __name__ == "__main__":
    simple_receiver_with_ctrl_c()
