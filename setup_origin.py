from pymavlink import mavutil
# master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
master.wait_heartbeat()
master.mav.set_gps_global_origin_send(
    master.target_system,
    int(35.000000 * 1e7),  # 緯度
    int(135.000000 * 1e7), # 経度
    0                      # 高度（mm単位、0でOK）
)
