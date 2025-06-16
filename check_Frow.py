from pymavlink import mavutil

# 115200bpsで一度接続してパラメータ確認
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200, rtscts=True)
master.wait_heartbeat()

# SERIAL1_BAUDの現在値を確認
master.mav.param_request_read_send(
    master.target_system,
    master.target_component,
    b'SERIAL1_BAUD',
    -1
)
msg = master.recv_match(type='PARAM_VALUE', blocking=True)
print(f"SERIAL1_BAUD = {msg.param_value}")

