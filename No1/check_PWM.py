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
    print("正規化スロットル値記録プログラム開始（デバッグ強化版）")
    print("=" * 70)
    
    # 接続確認の強化
    print("MAVLink接続中...")
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=1000000, rtscts=True)
    
    print("Heartbeat待機中...")
    master.wait_heartbeat(timeout=10)
    print(f"✅ Heartbeat受信: system={master.target_system}, component={master.target_component}")
    
    # データストリーム要求
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
    
    # デバッグカウンタ
    loop_count = 0
    message_count = 0
    norm_message_count = 0
    
    while running:
        loop_count += 1
        
        # 定期的な生存確認
        if loop_count % 100 == 0:
            print(f"[DEBUG] ループ回数: {loop_count}, 総メッセージ: {message_count}, NORM_AVG: {norm_message_count}")
        
        try:
            msg = master.recv_match(blocking=True, timeout=1)
            
            if msg is not None:
                message_count += 1
                current_time = datetime.now()
                msg_type = msg.get_type()
                
                # デバッグ: すべてのSTATUSTEXTを表示
                if msg_type == 'STATUSTEXT':
                    text = msg.text.strip()
                    print(f"[ALL_MSG] {text}")  # 全メッセージを確認
                    
                    # 正規化値データの処理
                    if "NORM_AVG:" in text:
                        norm_message_count += 1
                        norm_info = parse_norm_avg_data(text)
                        
                        if norm_info:
                            current_norm = norm_info['current']
                            hover_norm = norm_info['hover']
                            
                            print(f"✅ [NORM #{norm_message_count}] 現在値:{current_norm:.4f} ホバー基準:{hover_norm:.4f if hover_norm else 'N/A'}")
                        else:
                            print(f"❌ [PARSE_ERROR] {text}")
            
            else:
                # タイムアウト時の処理
                if loop_count % 10 == 0:  # 10秒ごと
                    print(f"[TIMEOUT] メッセージ待機中... (ループ#{loop_count})")
        
        except Exception as e:
            print(f"❌ [EXCEPTION] ループ内エラー: {e}")
            break

except KeyboardInterrupt:
    print("\n[INFO] ユーザーによる中断")

except Exception as e:
    print(f"❌ [FATAL ERROR] 致命的エラー: {e}")
    import traceback
    traceback.print_exc()

finally:
    try:
        if 'master' in locals():
            master.close()
            print("✅ MAVLink接続クローズ")
    except:
        pass
    
    print(f"\n統計情報:")
    print(f"  総ループ回数: {loop_count}")
    print(f"  受信メッセージ数: {message_count}")
    print(f"  NORM_AVGメッセージ数: {norm_message_count}")
    print("プログラム終了")
