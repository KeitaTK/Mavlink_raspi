from pymavlink import mavutil

print("115200bpsで接続してパラメータ変更")
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200, rtscts=True)
master.wait_heartbeat()

# SERIAL1_BAUDを921600に設定
master.mav.param_set_send(
    master.target_system,
    master.target_component,
    b'SERIAL1_BAUD',
    921600,
    mavutil.mavlink.MAV_PARAM_TYPE_REAL32
)

print("パラメータ設定完了。Pixhawkを再起動してください。")
master.close()
