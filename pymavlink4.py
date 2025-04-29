from pymavlink import mavutil

connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
connection.wait_heartbeat()

# ATTITUDE_QUATERNION (ID: 61) を10Hz（100ms間隔）で送信リクエスト
connection.mav.command_long_send(
    connection.target_system,
    connection.target_component,
    mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
    0,
    mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE_QUATERNION,  # 61
    100000,  # 100,000マイクロ秒 = 0.1秒 = 10Hz
    0, 0, 0, 0, 0
)
