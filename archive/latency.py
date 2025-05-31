from pymavlink import mavutil
import time
import statistics

def measure_timesync_latency(master, num_samples=10):
    latencies = []
    
    for i in range(num_samples):
        # 現在時刻をマイクロ秒で取得
        ts1 = int(time.time() * 1000000)
        
        # TIMESYNC送信
        master.mav.timesync_send(0, ts1)
        
        # レスポンス待機
        msg = master.recv_match(type='TIMESYNC', blocking=True, timeout=1)
        if msg and msg.ts1 == ts1:
            arrival_time = int(time.time() * 1000000)
            # 往復時間 / 2 = レイテンシ
            latency_us = (arrival_time - ts1) / 2
            latencies.append(latency_us / 1000)  # ミリ秒に変換
            print(f"Sample {i+1}: {latency_us/1000:.2f} ms")
        
        time.sleep(0.1)  # 100ms間隔
    
    if latencies:
        avg_latency = statistics.mean(latencies)
        min_latency = min(latencies)
        max_latency = max(latencies)
        print(f"\n平均レイテンシ: {avg_latency:.2f} ms")
        print(f"最小レイテンシ: {min_latency:.2f} ms")
        print(f"最大レイテンシ: {max_latency:.2f} ms")
    
    return latencies

# 使用例
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)
latencies = measure_timesync_latency(master, 20)
