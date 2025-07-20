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

def parse_norm_avg_data(text):
    try:
        if "NORM_AVG:" not in text:
            return None
        
        data_part = text.split("NORM_AVG:")[1].strip()
        
        if "(hover:" in data_part:
            current_part = data_part.split("(hover:")[0].strip()
            hover_part = data_part.split("(hover:")[1].replace(")", "").strip()
        else:
            current_part = data_part
            hover_part = None
        
        current_value = float(current_part)
        hover_value = float(hover_part) if hover_part else None
        
        return {
            'current': current_value,
            'hover': hover_value
        }
    except Exception as e:
        print(f"パースエラー: {e}, テキスト: {text}")
        return None

try:
    print("正規化スロットル値記録プログラム開始（修正版）")
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
    
    print("データ収集開始... (Ctrl+Cで終了)")
    print("=" * 70)
    
    # データ収集用変数
    norm_samples = []
    hover_values = []
    loop_count = 0
    message_count = 0
    norm_message_count = 0
    
    while running:
        loop_count += 1
        
        if loop_count % 100 == 0:
            print(f"[DEBUG] ループ回数: {loop_count}, 総メッセージ: {message_count}, NORM_AVG: {norm_message_count}")
        
        try:
            msg = master.recv_match(blocking=True, timeout=1)
            
            if msg is not None:
                message_count += 1
                current_time = datetime.now()
                msg_type = msg.get_type()
                
                if msg_type == 'STATUSTEXT':
                    text = msg.text.strip()
                    
                    if "NORM_AVG:" in text:
                        norm_message_count += 1
                        norm_info = parse_norm_avg_data(text)
                        
                        if norm_info:
                            current_norm = norm_info['current']
                            hover_norm = norm_info['hover']
                            
                            # データ収集
                            norm_samples.append(current_norm)
                            if hover_norm is not None:
                                hover_values.append(hover_norm)
                            
                            # 修正されたフォーマット（条件演算子を外に出す）
                            hover_text = f"{hover_norm:.4f}" if hover_norm is not None else "N/A"
                            print(f"✅ [NORM #{norm_message_count}] 現在値:{current_norm:.4f} ホバー基準:{hover_text}")
            
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
    print(f"  NORM_AVGメッセージ数: {norm_message_count}")
    
    if norm_samples:
        print("\n正規化スロットル値統計:")
        avg_norm = sum(norm_samples) / len(norm_samples)
        min_norm = min(norm_samples)
        max_norm = max(norm_samples)
        
        print(f"  平均正規化値: {avg_norm:.4f}")
        print(f"  最小値: {min_norm:.4f}")
        print(f"  最大値: {max_norm:.4f}")
        print(f"  変動範囲: {max_norm - min_norm:.4f}")
        
        if hover_values:
            avg_hover = sum(hover_values) / len(hover_values)
            deviation = avg_norm - avg_hover
            deviation_percent = (deviation / avg_hover) * 100 if avg_hover > 0 else 0
            
            print(f"  平均ホバー基準値: {avg_hover:.4f}")
            print(f"  ホバー基準からの偏差: {deviation:+.4f} ({deviation_percent:+.1f}%)")
    
    print("プログラム終了")
