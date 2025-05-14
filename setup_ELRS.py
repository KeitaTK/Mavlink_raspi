from pymavlink import mavutil
import time

# 接続を確立（実際の接続方法に合わせて変更してください）
# 例: シリアル接続 ('/dev/ttyACM0', baud=115200)
# 例: UDP接続 ('udpin:0.0.0.0:14550')  USB接続: 'COM3'など
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)

# ハートビートを待機してシステムIDを取得
print("Waiting for heartbeat...")
master.wait_heartbeat()
print(f"Heartbeat received from system (system {master.target_system} component {master.target_component})")

# パラメータを設定する関数
def set_param(param_id, param_value, param_type=mavutil.mavlink.MAV_PARAM_TYPE_REAL32):
    print(f"Setting {param_id} to {param_value}...")
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        param_id.encode('utf-8'),
        float(param_value),
        param_type
    )
    
    # ACKを待機
    start_time = time.time()
    timeout = 5
    while time.time() - start_time < timeout:
        message = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
        if message and message.param_id.decode('utf-8').strip('\x00') == param_id:
            print(f"Successfully set {param_id} to {message.param_value}")
            return True
    print(f"Failed to set {param_id} - no confirmation received")
    return False

# パラメータ設定
set_param('SERIAL2_PROTOCOL', 23)
set_param('RSSI_TYPE', 3)
rc_options_value = 8192 + 16384  # Suppress CRSFメッセージ + 420kbaudの設定
set_param('RC_OPTIONS', rc_options_value)

# パラメータをEEPROMに永続保存する
print("Saving parameters to EEPROM...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE,
    0,
    1, 0, 0, 0, 0, 0, 0
)

print("Command sent to save parameters. Waiting for confirmation...")

# 保存コマンドの確認応答を待機
start_time = time.time()
timeout = 5
while time.time() - start_time < timeout:
    msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
    if msg and msg.command == mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE:
        if msg.result == 0:
            print("Parameters saved successfully.")
        else:
            print(f"Failed to save parameters, result code: {msg.result}")
        break
else:
    print("No acknowledgement received for save command.")

print("Setup complete!")
