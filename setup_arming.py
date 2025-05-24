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
    'COM_DISARM_PREFE': 0,  # ディスアーム条件を緩和
    'MOT_SPIN_ARM': 0.05,
    'MOT_SPIN_MIN': 0.07
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
