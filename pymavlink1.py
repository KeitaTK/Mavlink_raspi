# mavkinkの情報を受信し、コンソールに表示する。
from pymavlink import mavutil
import math

conn = mavutil.mavlink_connection('udpin:localhost:14550')
hb = conn.wait_heartbeat()
print(f"System ID: {hb.get_srcSystem()}, Component ID: {hb.get_srcComponent()}")
print(f"Type: {mavutil.mavlink.enums['MAV_TYPE'][hb.type].name}")
print(f"Autopilot: {mavutil.mavlink.enums['MAV_AUTOPILOT'][hb.autopilot].name}")

while True:
    # 姿勢情報（ATTITUDE）
    msg = conn.recv_match(type='ATTITUDE', blocking=True)
    print(f"Roll: {math.degrees(msg.roll):.2f}°, Pitch: {math.degrees(msg.pitch):.2f}°")