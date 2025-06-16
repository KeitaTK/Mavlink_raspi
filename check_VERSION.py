from pymavlink import mavutil

# 1. シリアル接続の確立
#    接続先ポートと baudrate を設定
# USB接続の場合
# print("USB接続通信")
# master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
# telem1で接続するとき
print("tekem1通信接続")
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)

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