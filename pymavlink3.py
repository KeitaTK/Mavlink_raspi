from pymavlink import mavutil

connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
connection.wait_heartbeat()
print("Heartbeat received")

while True:
    msg = connection.recv_match(type='ATTITUDE_QUATERNION', blocking=True)
    if msg:
        # クオータニオン成分（w, x, y, z）
        w = msg.q1
        x = msg.q2
        y = msg.q3
        z = msg.q4
        print(f"Quaternion: w={w}, x={x}, y={y}, z={z}")

