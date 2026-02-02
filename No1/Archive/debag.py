from pymavlink import mavutil
import time
from datetime import datetime

def diagnostic():
    print("=" * 70)
    print("Pixhawk 6C 診断プログラム")
    print("=" * 70)
    
    try:
        print("\n[1/3] シリアル接続を試行...")
        master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200, timeout=1)
        print("✓ シリアル接続成功")
        
        print("\n[2/3] Heartbeat待機中... (最大5秒)")
        master.wait_heartbeat(timeout=5)
        print(f"✓ Heartbeat受信 - System: {master.target_system}, Component: {master.target_component}")
        
        print("\n[3/3] データストリーム要求...")
        master.mav.request_data_stream_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            10, 1
        )
        
        print("メッセージ受信待機中... (5秒間)")
        message_received = False
        
        for sec in range(5):
            msg = master.recv_match(blocking=False, timeout=0)
            if msg and msg.get_type() != 'HEARTBEAT':
                print(f"✓ データ受信: {msg.get_type()}")
                message_received = True
                break
            time.sleep(1)
            print(f"  [{sec+1}秒] 待機中...", end='\r')
        
        print("\n" + "=" * 70)
        if message_received:
            print("✓ 正常: ボードは正常にデータを送信しています")
            print("→ 問題: Mission Planner側のドライバまたはバージョン")
        else:
            print("✗ 警告: ボードからデータが返されていません")
            print("→ 可能性1: ブートローダーモード")
            print("→ 可能性2: カスタムファームウェアがクラッシュ")
            print("→ 可能性3: MAVLink初期化未完了")
        
        master.close()
        
    except Exception as e:
        print(f"✗ エラー: {e}")
        print("→ 考えられる原因:")
        print("   - ボードが接続されていない")
        print("   - ポート設定が間違っている（/dev/ttyACM0が存在しない）")
        print("   - ドライバがインストールされていない")

if __name__ == "__main__":
    diagnostic()
