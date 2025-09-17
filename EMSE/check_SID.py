from pymavlink import mavutil
import time

# Mode constants
MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
MAV_MODE_FLAG_SAFETY_ARMED       = 128
SYSTEM_ID_MODE                   = 25  # custom_mode for System Identification

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

            # 4. システムIDモードに切り替え（まだ切り替わっていないなら）
            if mode_id != SYSTEM_ID_MODE:
                master.mav.command_long_send(
                    master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                    0,
                    MAV_MODE_FLAG_CUSTOM_MODE_ENABLED + MAV_MODE_FLAG_SAFETY_ARMED,
                    SYSTEM_ID_MODE,
                    0, 0, 0, 0, 0
                )
                print("Requested switch to System Identification mode")
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
