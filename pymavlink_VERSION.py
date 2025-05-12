from pymavlink import mavutil

# 1. シリアル接続の確立
#    接続先ポートと baudrate を設定
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)

# 2. heartbeat 受信待ち
#    これにより target_system, target_component が設定される
master.wait_heartbeat()
print(f"Heartbeat received from system {master.target_system}, component {master.target_component}")

# 3. AUTOPILOT_VERSION_REQUEST の送信
#    メソッド名は mavlink 定義に準拠
master.mav.autopilot_version_request_send(
    master.target_system,
    master.target_component
)

# 4. AUTOPILOT_VERSION メッセージの受信
#    blocking=True で到着まで待機
msg = master.recv_match(type='AUTOPILOT_VERSION', blocking=True)
if msg:
    data = msg.to_dict()
    print("AUTOPILOT_VERSION received:")
    for k, v in data.items():
        print(f"  {k}: {v}")


# from pymavlink import mavutil

# # 1. シリアル接続の確立
# #    接続先ポートと baudrate を設定
# master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)

# # 2. heartbeat 受信待ち
# #    これにより target_system, target_component が設定される
# master.wait_heartbeat()
# print(f"Heartbeat received from system {master.target_system}, component {master.target_component}")

# # 3. AUTOPILOT_VERSION_REQUEST2 の送信
# #    カスタムメッセージ用の送信メソッドを使用
# master.mav.autopilot_version_request2_send(
#     master.target_system,
#     master.target_component
# )

# # 4. AUTOPILOT_VERSION メッセージの受信
# #    応答はおそらく標準の AUTOPILOT_VERSION として返ってくる
# msg = master.recv_match(type='AUTOPILOT_VERSION', blocking=True, timeout=10)
# if msg:
#     data = msg.to_dict()
#     print("AUTOPILOT_VERSION received:")
#     for k, v in data.items():
#         print(f"  {k}: {v}")
# else:
#     print("タイムアウト: AUTOPILOT_VERSION メッセージを受信できませんでした")
    
#     # デバッグ: 実際に受信したメッセージを確認
#     print("受信したその他のメッセージ:")
#     for i in range(5):  # 5つのメッセージを表示
#         any_msg = master.recv_match(blocking=True, timeout=2)
#         if any_msg:
#             print(f"  {any_msg.get_type()}: {any_msg.to_dict()}")
#         else:
#             break
