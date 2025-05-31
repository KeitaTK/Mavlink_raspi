from pymavlink import mavutil
import math
import sys

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
    try:
        msg = connection.recv_match(blocking=True)
        if msg:
            if msg.get_type() == 'ATTITUDE':
                # ラジアンから度数へ変換
                roll_deg = math.degrees(msg.roll)
                pitch_deg = math.degrees(msg.pitch)
                yaw_deg = math.degrees(msg.yaw)
                
                # フォーマットして表示
                print(f"\n[姿勢情報]")
                print(f"ロール角: {roll_deg:6.2f}°")
                print(f"ピッチ角: {pitch_deg:6.2f}°")
                print(f"ヨー角: {yaw_deg:6.2f}°")
                print("-" * 30)
            else:
                print(f"[その他メッセージ] {msg}")
    except KeyboardInterrupt:  # Ctrl+Cが押されたら
        sys.exit()
