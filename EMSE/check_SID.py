from pymavlink import mavutil
import time

try:
    # 1. シリアル接続の確立（ハードウェアフロー制御有効）
    print("TELEM1通信接続（ハードウェアフロー制御有効）")
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=1000000, rtscts=True)

    # 2. heartbeat 受信待ち（タイムアウト設定）
    master.wait_heartbeat(timeout=5)
    print(f"Heartbeat received from system {master.target_system}, component {master.target_component}")

    while True:
        # 3. HEARTBEATメッセージの受信（非ブロッキング）
        msg = master.recv_match(type='HEARTBEAT', blocking=False)
        if msg:
            mode_id = msg.custom_mode
            base_mode = msg.base_mode
            print(f"Current Mode ID: {mode_id}, Base Mode: {base_mode}")
        else:
            print("No HEARTBEAT message received")

        time.sleep(1)  # 1秒間隔で繰り返し

except Exception as e:
    print(f"通信エラー: {e}")

finally:
    try:
        master.close()
    except:
        pass
