from pymavlink import mavutil

# USB接続の場合、/dev/ttyACM0を指定
connection = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
connection.wait_heartbeat()
print("Heartbeat received")

while True:
    msg = connection.recv_match(blocking=True)
    if msg:
        print(msg)
