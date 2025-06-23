from pymavlink import mavutil
import time

# シリアル接続を確立
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)

# ハートビートを待ってシステムIDとコンポーネントIDを設定
master.wait_heartbeat()
print("Heartbeat from system (system %u component %u)" % (master.target_system, master.target_component))

# LOCAL_POSITION_NEDメッセージのストリーミングを要求
# MAV_CMD_SET_MESSAGE_INTERVALコマンドを使用
message = master.mav.command_long_encode(
    master.target_system,                                    # ターゲットシステムID
    master.target_component,                                 # ターゲットコンポーネントID
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,           # コマンドID
    0,                                                       # 確認フラグ
    mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED,      # param1: メッセージID
    100000,                                                  # param2: 間隔（マイクロ秒）- 10Hz
    0, 0, 0, 0, 0                                           # param3-7 (未使用)
)

# コマンドを送信
master.mav.send(message)

# 応答を待つ（オプション）
response = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
if response and response.command == mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL:
    if response.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("LOCAL_POSITION_NED streaming command accepted")
    else:
        print(f"Command failed with result: {response.result}")

# LOCAL_POSITION_NEDメッセージを受信
while True:
    msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True)
    if msg:
        print(f"Position - X: {msg.x:.2f}, Y: {msg.y:.2f}, Z: {msg.z:.2f}")
        print(f"Velocity - VX: {msg.vx:.2f}, VY: {msg.vy:.2f}, VZ: {msg.vz:.2f}")
