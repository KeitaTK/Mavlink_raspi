from pymavlink import mavutil
import math  # 追加

# USB接続の場合、/dev/ttyACM0を指定
connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
connection.wait_heartbeat()
print("Heartbeat received")

# 姿勢データの送信間隔設定（10Hz）
connection.mav.command_long_send(
    connection.target_system,
    connection.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    0,
    mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
    100000,  # 100000μs = 0.1秒間隔
    0, 0, 0, 0, 0
)

while True:
    msg = connection.recv_match(type='ATTITUDE_QUATERNION', blocking=True)
    if msg:
        # クオータニオン成分（w, x, y, z）
        w = msg.q1
        x = msg.q2
        y = msg.q3
        z = msg.q4
        print(f"Quaternion: w={w}, x={x}, y={y}, z={z}")
