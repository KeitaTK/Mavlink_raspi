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
    # 1. シリアル接続の確立（元の設定を維持）
    print("TELEM1通信接続（ハードウェアフロー制御有効）")
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=1000000, rtscts=True)

    # 2. heartbeat 受信待ち（タイムアウト設定）
    master.wait_heartbeat(timeout=5)
    print(f"Heartbeat received from system {master.target_system}, component {master.target_component}")

    # 3. AUTOPILOT_VERSION_REQUEST の送信（元のまま）
    master.mav.autopilot_version_request_send(
        master.target_system,
        master.target_component
    )

    # 4. AUTOPILOT_VERSION メッセージの受信（元のまま）
    msg = master.recv_match(type='AUTOPILOT_VERSION', blocking=True, timeout=3)
    if msg:
        data = msg.to_dict()
        print("AUTOPILOT_VERSION received:")
        for k, v in data.items():
            print(f"  {k}: {v}")
    else:
        print("AUTOPILOT_VERSION メッセージの受信に失敗しました（タイムアウト）")

    # 5. STATUSTEXTメッセージの受信を要求（追加）
    print("\nSTATUSTEXTメッセージの受信を要求しています...")
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
        1,  # 1Hz
        1   # start
    )

    # 6. 継続的なメッセージ受信（追加）
    print("継続的なメッセージ受信を開始... (Ctrl+C で停止)")
    print("-" * 50)
    
    while running:
        # すべてのメッセージを受信
        msg = master.recv_match(blocking=True, timeout=1)
        
        if msg is not None:
            # 現在の時刻を取得
            current_time = datetime.now().strftime("%Y/%m/%d %H:%M:%S")
            msg_type = msg.get_type()
            
            if msg_type == 'STATUSTEXT':
                # ステータステキストメッセージ（ログメッセージ）を表示
                text = msg.text.strip()
                print(f"{current_time} : {text}")
                
            elif msg_type == 'HEARTBEAT':
                # ハートビートメッセージ（表示しない）
                pass
                
            elif msg_type == 'SYSTEM_TIME':
                # システム時刻メッセージ（表示しない）
                pass
                
            elif msg_type == 'TIMESYNC':
                # タイムシンクメッセージ（表示しない）
                pass
                
            elif msg_type == 'BAD_DATA':
                # BAD_DATAエラーがあれば表示
                print(f"{current_time} : [警告] データ受信エラー")
                
            # その他のメッセージは表示しない（ノイズ軽減）
        
        # CPU負荷軽減のため少し待機
        time.sleep(0.01)

except KeyboardInterrupt:
    print("\nユーザーによって中断されました")
    
except Exception as e:
    print(f"通信エラー: {e}")

finally:
    try:
        if 'master' in locals():
            master.close()
    except:
        pass
    print("プログラムを終了しました")
