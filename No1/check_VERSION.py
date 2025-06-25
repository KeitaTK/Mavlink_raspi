# from pymavlink import mavutil


# # master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)   # USB
# master = mavutil.mavlink_connection('/dev/ttyAMA0', 921600, rtscts=True)  # フロー制御
# # 2. heartbeat 受信待ち
# #    これにより target_system, target_component が設定される
# master.wait_heartbeat()
# print(f"Heartbeat received from system {master.target_system}, component {master.target_component}")

# # 3. AUTOPILOT_VERSION_REQUEST の送信
# #    メソッド名は mavlink 定義に準拠
# master.mav.autopilot_version_request_send(
#     master.target_system,
#     master.target_component
# )

# # 4. AUTOPILOT_VERSION メッセージの受信
# #    blocking=True で到着まで待機
# msg = master.recv_match(type='AUTOPILOT_VERSION', blocking=True)
# if msg:
#     data = msg.to_dict()
#     print("AUTOPILOT_VERSION received:")
#     for k, v in data.items():
#         print(f"  {k}: {v}")


from pymavlink import mavutil
import time

try:
    # 1. シリアル接続の確立（ハードウェアフロー制御有効）
    print("TELEM1通信接続（ハードウェアフロー制御有効）")
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=921600, rtscts=True)

    # 2. heartbeat 受信待ち（タイムアウト設定）
    master.wait_heartbeat(timeout=5)
    print(f"Heartbeat received from system {master.target_system}, component {master.target_component}")

    # 3. AUTOPILOT_VERSION_REQUEST の送信
    master.mav.autopilot_version_request_send(
        master.target_system,
        master.target_component
    )

    # 4. AUTOPILOT_VERSION メッセージの受信（タイムアウト設定）
    msg = master.recv_match(type='AUTOPILOT_VERSION', blocking=True, timeout=3)
    if msg:
        data = msg.to_dict()
        print("AUTOPILOT_VERSION received:")
        for k, v in data.items():
            print(f"  {k}: {v}")
    else:
        print("AUTOPILOT_VERSION メッセージの受信に失敗しました（タイムアウト）")

except Exception as e:
    print(f"通信エラー: {e}")

finally:
    try:
        master.close()
    except:
        pass
