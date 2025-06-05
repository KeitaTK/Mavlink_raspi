import time
from pymavlink import mavutil

# MAVLinkコネクション
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
master.wait_heartbeat()
print(f"接続確認: システム {master.target_system}")

# 最新版対応パラメータ
params = {
    'PSC_POSZ_P': 2.0,
    'PSC_VELZ_P': 8.0,
    'PSC_VELZ_I': 15.0,
    'PSC_VELZ_D': 0.02,
    'PSC_ACCZ_P': 1.0,
    'PSC_ACCZ_I': 2.0,
    'WPNAV_SPEED_UP': 300,
    'WPNAV_SPEED_DN': 150,
    'WPNAV_ACCEL_Z': 300,
    'PILOT_SPEED_UP': 250,
    'PILOT_SPEED_DN': 150,
    'PILOT_ACCEL_Z': 250,
    'INS_GYRO_FILTER': 20,
    'INS_ACCEL_FILTER': 20,
}

def safe_param_name(param_id):
    try:
        if isinstance(param_id, bytes):
            return param_id.decode('utf-8').rstrip('\x00')
        else:
            return str(param_id).rstrip('\x00')
    except:
        return str(param_id)

print("高度制御パラメータ設定開始")
print("-" * 50)

success_count = 0
for param_id, param_value in params.items():
    print(f"{param_id}を{param_value}に設定中...")
    
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        param_id.encode('utf-8'),
        param_value,
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )
    
    message = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
    if message:
        param_name = safe_param_name(message.param_id)
        if param_name.upper() == param_id.upper():
            print(f"確認: {param_name} = {message.param_value}")
            success_count += 1
        else:
            retry_msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
            if retry_msg and safe_param_name(retry_msg.param_id).upper() == param_id.upper():
                print(f"確認: {safe_param_name(retry_msg.param_id)} = {retry_msg.param_value}")
                success_count += 1
            else:
                print(f"タイムアウト: {param_id}")
    else:
        print(f"タイムアウト: {param_id}")
    
    time.sleep(0.3)

print("-" * 50)
print("パラメータをEEPROMに保存中...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE,
    0, 1, 0, 0, 0, 0, 0, 0
)

ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
if ack_msg and ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
    print("パラメータ保存完了")
else:
    print("保存結果不明")

print(f"設定完了: {success_count}/{len(params)} 成功")
