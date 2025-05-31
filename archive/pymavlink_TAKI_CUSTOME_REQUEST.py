# from pymavlink import mavutil

# # 1. シリアル接続の確立
# #    接続先ポートと baudrate を設定
# master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)

# # 2. heartbeat 受信待ち
# #    これにより target_system, target_component が設定される
# master.wait_heartbeat()
# print(f"Heartbeat received from system {master.target_system}, component {master.target_component}")

# # # 3. AUTOPILOT_VERSION_REQUEST の送信
# # #    メソッド名は mavlink 定義に準拠
# # master.mav.autopilot_version_request_send(
# #     master.target_system,
# #     master.target_component
# # )

# master.mav.taki_custome1_request_send(
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


# from pymavlink import mavutil
# import time

# master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)

# # カスタムメッセージリクエスト送信
# master.mav.taki_custome1_request_send(
#     master.target_system,
#     master.target_component
# )
# print("TAKI_CUSTOME1_REQUEST sent")

# # TAKI_CUSTOME1メッセージのみを監視
# start_time = time.time()
# while time.time() - start_time < 5:
#     msg = master.recv_match(type='TAKI_CUSTOME1', blocking=False)
#     if msg:
#         print(f"TAKI_CUSTOME1 received - counter={msg.test_counter}")
#         break  # 受信したら終了
#     time.sleep(0.1)
# else:
#     print("No TAKI_CUSTOME1 message received within 5 seconds")


from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)

for i in range(3):
    print(f"\n--- Test {i+1} ---")
    
    # リクエスト送信
    master.mav.taki_custome1_request_send(
        master.target_system,
        master.target_component
    )
    print("Request sent")
    
    # レスポンス受信
    msg = master.recv_match(type='TAKI_CUSTOME1', blocking=True, timeout=3)
    if msg:
        print(f"Response: counter={msg.test_counter}")
    else:
        print("No response")
    
    time.sleep(1)
