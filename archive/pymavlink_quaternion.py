from pymavlink import mavutil
import sys

connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
connection.wait_heartbeat()
print("Heartbeat received")

# ATTITUDE_QUATERNION (ID: 61) を10Hz（100ms間隔）で送信リクエスト
connection.mav.command_long_send(
    connection.target_system,
    connection.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    0,
    mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION,  # 61
    10000,  # 100,000マイクロ秒 = 0.1秒 = 10Hz
    0, 0, 0, 0, 0
)


while True:
    try:
        msg = connection.recv_match(type='ATTITUDE_QUATERNION', blocking=True)
        if msg:
            # クオータニオン成分（w, x, y, z）
            w = msg.q1
            x = msg.q2
            y = msg.q3
            z = msg.q4
            print(f"Quaternion: w={w}, x={x}, y={y}, z={z}")

    except KeyboardInterrupt:  # Ctrl+Cが押されたら
        sys.exit()