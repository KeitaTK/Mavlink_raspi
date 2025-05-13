from pymavlink import mavutil
import time

# 接続を確立（実際の接続方法に合わせて変更してください）
# 例: シリアル接続 ('/dev/ttyACM0', baud=115200)
# 例: UDP接続 ('udpin:0.0.0.0:14550')  USB接続: 'COM3'など
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)

# ハートビートを待機（接続確認）
print("接続を待機中...")
master.wait_heartbeat()
print(f"接続完了 (システム: {master.target_system}, コンポーネント: {master.target_component})")

# DShot300に設定するパラメータ
params_to_set = {
    # MAINチャンネルを無効化（必要に応じて）
    'SERVO1_FUNCTION': 0,  # 無効化
    'SERVO2_FUNCTION': 0,  # 無効化
    'SERVO3_FUNCTION': 0,  # 無効化
    'SERVO4_FUNCTION': 0,  # 無効化
    
    # AUXチャンネル（SERVO9-12）をモーター出力に設定
    'SERVO9_FUNCTION': 33,  # モーター1
    'SERVO10_FUNCTION': 34, # モーター2
    'SERVO11_FUNCTION': 35, # モーター3
    'SERVO12_FUNCTION': 36, # モーター4

    'MOT_PWM_TYPE': 4,  # 4=DShot300
    'SERVO_DSHOT_ESC': 2,  # BLHeli_S/BlueJayの場合は2を設定
    'SERVO_BLH_MASK': 3840,  # チャンネル9,10,11,12に対応
    'SERVO_BLH_AUTO': 1,   # パススルー有効化（BLHeliSuiteでESCを設定する場合）
}


# パラメータを設定
print("DShot300パラメータを設定中...")
for param_name, param_value in params_to_set.items():
    # パラメータを設定（一時的にRAMに保存）
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        param_name.encode('utf-8'),
        float(param_value),
        mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    )
    
    # 確認メッセージを待機
    message = master.recv_match(type='PARAM_VALUE', blocking=True).to_dict()
    print(f'パラメータ設定: {param_name} = {message["param_value"]}')
    time.sleep(0.5)

# 設定をEEPROMに永続的に保存するコマンドを送信
print("設定をEEPROMに保存中...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE,
    0,  # 確認パラメータ
    1,  # 1=Write parameters（設定を保存）
    0, 0, 0, 0, 0, 0  # 未使用のパラメータ
)

# コマンドACKを待機
ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
if ack and ack.command == mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
    print("EEPROMへの保存成功")
else:
    print("EEPROMへの保存でエラーまたはタイムアウトが発生")

# 確認のため、パラメータを再読み込み
print("設定を確認中...")
for param_name in params_to_set.keys():
    master.mav.param_request_read_send(
        master.target_system,
        master.target_component,
        param_name.encode('utf-8'),
        -1  # -1はインデックスではなく名前でパラメータを取得
    )
    
    message = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=2).to_dict()
    if message:
        print(f'確認: {param_name} = {message["param_value"]}')
    else:
        print(f'確認: {param_name} のデータを取得できませんでした')
    time.sleep(0.5)

print("AUXチャンネル1-4のモーター出力設定完了")
print("DShot300設定と保存の確認完了")
