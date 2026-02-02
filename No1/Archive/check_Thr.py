from pymavlink import mavutil
import time
import signal
import sys
from datetime import datetime

running = True

def signal_handler(sig, frame):
    global running
    print('\n終了中...')
    running = False

signal.signal(signal.SIGINT, signal_handler)

def parse_throttle_data(text):
    """
    THROTTLE: 0.3785 の形式からスロットル値を抽出
    """
    try:
        if "THROTTLE:" not in text:
            return None
        
        # "THROTTLE: " 以降を取得
        throttle_part = text.split("THROTTLE:")[1].strip()
        throttle_value = float(throttle_part)
        
        return throttle_value
    except Exception as e:
        print(f"パースエラー: {e}, テキスト: {text}")
        return None

try:
    print("スロットル値記録プログラム開始")
    print("=" * 70)
    
    print("MAVLink接続中...")
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=1000000, rtscts=True)
    
    print("Heartbeat待機中...")
    master.wait_heartbeat(timeout=10)
    print(f"✅ Heartbeat受信: system={master.target_system}, component={master.target_component}")
    
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
        10,
        1
    )
    print("✅ データストリーム要求送信")
    
    print("スロットルデータ収集中... (Ctrl+Cで終了)")
    print("=" * 70)
    
    # データ収集用変数
    throttle_samples = []
    loop_count = 0
    message_count = 0
    throttle_message_count = 0
    
    while running:
        loop_count += 1
        
        if loop_count % 100 == 0:
            print(f"[DEBUG] ループ回数: {loop_count}, 総メッセージ: {message_count}, THROTTLE: {throttle_message_count}")
        
        try:
            msg = master.recv_match(blocking=True, timeout=1)
            
            if msg is not None:
                message_count += 1
                current_time = datetime.now()
                msg_type = msg.get_type()
                
                if msg_type == 'STATUSTEXT':
                    text = msg.text.strip()
                    
                    if "THROTTLE:" in text:
                        throttle_message_count += 1
                        throttle_value = parse_throttle_data(text)
                        
                        if throttle_value is not None:
                            # データ収集
                            throttle_samples.append(throttle_value)
                            
                            print(f"✅ [THROTTLE #{throttle_message_count}] 値:{throttle_value:.4f}")
            
            else:
                if loop_count % 10 == 0:
                    print(f"[TIMEOUT] メッセージ待機中... (ループ#{loop_count})")
        
        except Exception as e:
            print(f"❌ [EXCEPTION] ループ内エラー: {e}")
            break

except KeyboardInterrupt:
    print("\n[INFO] ユーザーによる中断")

except Exception as e:
    print(f"❌ [FATAL ERROR] 致命的エラー: {e}")

finally:
    try:
        if 'master' in locals():
            master.close()
            print("✅ MAVLink接続クローズ")
    except:
        pass
    
    # 統計結果表示
    print(f"\n統計情報:")
    print(f"  総ループ回数: {loop_count}")
    print(f"  受信メッセージ数: {message_count}")
    print(f"  THROTTLEメッセージ数: {throttle_message_count}")
    
    if throttle_samples:
        print("\nスロットル値統計:")
        print("-" * 50)
        
        # 基本統計
        avg_throttle = sum(throttle_samples) / len(throttle_samples)
        min_throttle = min(throttle_samples)
        max_throttle = max(throttle_samples)
        throttle_range = max_throttle - min_throttle
        
        print(f"  サンプル数: {len(throttle_samples)}")
        print(f"  平均スロットル値: {avg_throttle:.4f}")
        print(f"  最小値: {min_throttle:.4f}")
        print(f"  最大値: {max_throttle:.4f}")
        print(f"  変動範囲: {throttle_range:.4f}")
        
        # 標準偏差と変動係数
        variance = sum([(x - avg_throttle)**2 for x in throttle_samples]) / len(throttle_samples)
        std_dev = variance ** 0.5
        cv = (std_dev / avg_throttle) * 100 if avg_throttle > 0 else 0
        
        print(f"  標準偏差: {std_dev:.4f}")
        print(f"  変動係数: {cv:.1f}%")
        
        # データ安定性評価
        print("\nデータ安定性評価:")
        print("-" * 50)
        if cv < 2:
            print("  → 非常に安定したスロットル制御")
        elif cv < 5:
            print("  → 安定したスロットル制御")
        elif cv < 10:
            print("  → やや変動のあるスロットル制御")
        else:
            print("  → 大きな変動のあるスロットル制御")
        
        # 測定時間推定
        estimated_time = len(throttle_samples) / 40  # 40Hz推定
        print(f"\n測定時間（推定）: {estimated_time:.1f}秒")
        
    else:
        print("\nスロットルデータが取得できませんでした。")
        print("AP_Observerが正常に動作しているか確認してください。")
    
    print("\nプログラム終了")
