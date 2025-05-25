import time
from pymavlink import mavutil

# MAVLinkコネクションを作成
# 注: UDPポートは環境に合わせて変更してください
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)

# コマンドを送信する前にハートビートを待つ
print("ハートビートを待機中...")
master.wait_heartbeat()
print(f"ハートビート受信: システム {master.target_system} コンポーネント {master.target_component}")

# 設定するパラメータとその値
parameters = {
    'BRD_SAFETY_DEFLT': 0,
    'BRD_SAFETYOPTION': 0,
    'ARMING_CHECK': 80,
    'ARMING_RUDDER': 0,
    'RC9_OPTION': 153,
    'DISARM_DELAY': 0,  # ディスアームの遅延を無効化（即時ディスアーム）
    'MOT_SPIN_ARM': 0.02,
    'MOT_SPIN_MIN': 0.02,
    # バッテリーモニターを有効化（電圧＋電流センサー）
    'BATT_MONITOR': 4,

    # 5セルLiIon用電圧設定
    'BATT_ARM_VOLT': 19.5,    # アーム可能電圧
    'BATT_CRT_VOLT': 14.0,    # 危険電圧
    'BATT_LOW_VOLT': 15.5,    # 警告電圧

    # モーター出力電圧範囲
    'MOT_BAT_VOLT_MAX': 21.0,  # 最大電圧
    'MOT_BAT_VOLT_MIN': 13.5,  # 最小電圧
    
# PID設定
    'ATC_RAT_RLL_P': 0.08,
    'ATC_RAT_RLL_I': 0.08,
    'ATC_RAT_RLL_D': 0.002,
    
    # Pitch軸
    'ATC_RAT_PIT_P': 0.16,
    'ATC_RAT_PIT_I': 0.16,
    'ATC_RAT_PIT_D': 0.004,
    
    # Yaw軸
    'ATC_RAT_YAW_P': 0.20,
    'ATC_RAT_YAW_I': 0.020,

}

# すべてのパラメータを設定
for param_id, param_value in parameters.items():
    print(f"{param_id}を{param_value}に設定中...")
    
    # パラメータを設定
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        param_id.encode('utf-8'),  # パラメータ名をバイト列に変換
        param_value,
        mavutil.mavlink.MAV_PARAM_TYPE_INT32  # 今回は整数値のみ使用
    )
    
    # ACK（応答）を読み取る
    message = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=5)
    if message:
        message_dict = message.to_dict()
        print(f"確認: {message_dict['param_id']} = {message_dict['param_value']}")
    else:
        print(f"警告: {param_id}の設定確認タイムアウト")
    
    # 次のコマンドの前に少し待機
    time.sleep(0.5)

# パラメータをEEPROMに永続保存する
print("Saving parameters to EEPROM...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE,
    0,
    1, 0, 0, 0, 0, 0, 0
)

print("すべてのパラメータが設定されました")
