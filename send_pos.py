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

class LatestDataBuffer:
    def __init__(self):
        self.latest_data = None
        self.data_count = 0
    
    def update(self, data):
        self.latest_data = data
        self.data_count += 1
    
    def get_latest(self):
        return self.latest_data
    
    def get_count(self):
        return self.data_count

def get_latest_udp_data(sock):
    """UDPソケットから最新データのみを取得（検索結果[5]を参考）"""
    sock.setblocking(False)  # ノンブロッキングモードに設定
    
    newest_data = None
    packets_read = 0
    
    while True:
        try:
            data_bytes, sender_addr = sock.recvfrom(1024)
            if data_bytes:
                newest_data = data_bytes
                packets_read += 1
        except socket.error as e:
            # EWOULDBLOCK または EAGAIN でデータが無い場合
            if e.errno == socket.errno.EWOULDBLOCK or e.errno == socket.errno.EAGAIN:
                break
            else:
                raise e
    
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

def receiver_with_20hz_control():
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
        # USB接続の場合
        # master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
        # telem1で接続するとき
        master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
        master.wait_heartbeat()
        print("✓ MAVLink connection established")
    except Exception as e:
        print(f"✗ MAVLink connection failed: {e}")
        master = None
    
    # 20Hz送信制御とデータバッファ
    freq_controller = FrequencyController(target_hz=20)
    data_buffer = LatestDataBuffer()
    
    print("Receiving 50Hz Motive data, sending at 20Hz...")
    print("Press Ctrl+C to stop")
    print("-" * 70)
    
    packet_count = 0
    mavlink_send_count = 0
    skipped_packets = 0
    
    try:
        while True:
            try:
                # 50HzのUDPデータから最新データのみを取得
                newest_data, packets_read = get_latest_udp_data(sock)
                
                if newest_data:
                    # データ処理
                    data = pickle.loads(newest_data)
                    pos = data['position']    # [x, y, z]
                    quat = data['quaternion'] # [w, x, y, z]
                    
                    # 最新データをバッファに保存
                    data_buffer.update({'pos': pos, 'quat': quat})
                    packet_count += 1
                    
                    if packets_read > 1:
                        skipped_packets += packets_read - 1
                    
                    # 20Hz制御で送信判定
                    if freq_controller.should_send() and master is not None:
                        latest_data = data_buffer.get_latest()
                        if latest_data:
                            try:
                                # MAVLink送信
                                time_usec = int(time.time() * 1000000)
                                
                                master.mav.att_pos_mocap_send(
                                    time_usec,              # time_usec
                                    latest_data['quat'],    # q (w,x,y,z)
                                    latest_data['pos'][0],  # x
                                    latest_data['pos'][1],  # y
                                    latest_data['pos'][2]   # z
                                )
                                
                                mavlink_send_count += 1
                                
                                # 送信確認表示
                                print(f"[{mavlink_send_count:03d}] MAVLink → "
                                      f"Pos: ({latest_data['pos'][0]:+6.2f}, {latest_data['pos'][1]:+6.2f}, {latest_data['pos'][2]:+6.2f}) | "
                                      f"RX: {packet_count} pkts | Skipped: {skipped_packets}")
                                
                            except Exception as e:
                                print(f"✗ MAVLink error: {e}")
                
                # 短時間スリープ（CPUリソース節約）
                time.sleep(0.001)  # 1ms
                
            except Exception as e:
                print(f"Data processing error: {e}")
                time.sleep(0.01)
                
    except Exception as e:
        print(f"Main loop error: {e}")
    finally:
        sock.close()
        if master:
            master.close()
        
        # 統計情報表示
        print(f"\nStatistics:")
        print(f"  Total UDP packets received: {packet_count}")
        print(f"  Total packets skipped: {skipped_packets}")
        print(f"  MAVLink messages sent: {mavlink_send_count}")
        print(f"  Effective send rate: {mavlink_send_count / (packet_count / 50.0):.1f} Hz")

if __name__ == "__main__":
    receiver_with_20hz_control()
