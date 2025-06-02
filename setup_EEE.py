from pymavlink import mavutil
import time

master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
master.wait_heartbeat()

params = {
    'EK3_SRC1_POSXY': 6.0,
    'EK3_SRC1_VELXY': 0.0,
    'EK3_SRC1_POSZ':  6.0,
    'EK3_SRC1_VELZ':  0.0,
    'EK3_SRC1_YAW':   6.0,
}

for name, val in params.items():
    master.mav.param_set_send(
        master.target_system, master.target_component,
        name.encode('utf-8'),
        val,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )
    time.sleep(0.2)

# EEPROMに保存＆リブート
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE,
    0,1,0,0,0,0,0,0
)
time.sleep(1)
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN,
    0,1,0,0,0,0,0,0
)
master.close()
