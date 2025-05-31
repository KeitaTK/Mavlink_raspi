import socket
import pickle

sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('', 15769))

print("Debug receiver - checking data format...")

try:
    data_bytes, addr = sock.recvfrom(1024)
    data = pickle.loads(data_bytes)
    
    print(f"Type: {type(data)}")
    print(f"Content: {data}")
    
    # 各キーをチェック（辞書の場合）
    if isinstance(data, dict):
        print(f"Keys: {list(data.keys())}")
        
except Exception as e:
    print(f"Error: {e}")
finally:
    sock.close()
