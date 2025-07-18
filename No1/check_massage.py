from pymavlink import mavutil
import time
import signal
import sys
from datetime import datetime

# グローバル変数で実行状態を管理
running = True

def signal_handler(sig, frame):
    global running
    print('\n終了中...')
    running = False

# Ctrl+C で安全に終了するためのシグナルハンドラ
signal.signal(signal.SIGINT, signal_handler)

try:
    # 1. シリアル接続の確立（ハードウェアフロー制御有効）
    print("TELEM1通信接続（ハードウェアフロー制御有効）")
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=1000000, rtscts=True)

    # 2. heartbeat 受信待ち（タイムアウト設定）
    master.wait_heartbeat(timeout=5)
    print(f"Heartbeat received from system {master.target_system}, component {master.target_component}")
    
    # 3. ログメッセージの受信を開始
    print("メッセージ受信を開始しています... (Ctrl+C で停止)")
    print("-" * 50)
    
    # 4. 継続的にメッセージを受信
    while running:
        # すべてのメッセージを受信（タイムアウト設定で無限待機を防ぐ）
        msg = master.recv_match(blocking=True, timeout=1)
        
        if msg is not None:
            # 現在の時刻を取得
            current_time = datetime.now().strftime("%Y/%m/%d %H:%M:%S")
            
            # メッセージタイプによって処理を分岐
            if msg.get_type() == 'STATUSTEXT':
                # ステータステキストメッセージ（ログメッセージ）を表示
                text = msg.text.strip()
                severity = msg.severity
                print(f"{current_time} : {text}")
                
            elif msg.get_type() == 'HEARTBEAT':
                # ハートビートメッセージ（必要に応じて表示）
                # print(f"{current_time} : Heartbeat from system {msg.get_srcSystem()}")
                pass
                
            elif msg.get_type() == 'SYSTEM_TIME':
                # システム時刻メッセージ
                pass
                
            elif msg.get_type() == 'AUTOPILOT_VERSION':
                # オートパイロットバージョン情報
                print(f"{current_time} : AUTOPILOT_VERSION received")
                
            else:
                # その他のメッセージ（必要に応じて表示）
                print(f"{current_time} : {msg.get_type()}: {msg}")
        
        # 少し待機（CPU負荷軽減）
        time.sleep(0.01)

except Exception as e:
    print(f"通信エラー: {e}")

finally:
    try:
        print("接続を閉じています...")
        master.close()
    except:
        pass
    print("プログラムを終了しました")
