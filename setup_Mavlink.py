import time
from pymavlink import mavutil

# MAVLinkコネクションを作成
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)

# コマンドを送信する前にハートビートを待つ
print("ハートビートを待機中...")
master.wait_heartbeat()
print(f"ハートビート受信: システム {master.target_system} コンポーネント {master.target_component}")

# 設定するパラメータとその値（修正版）
parameters = {
    'SERIAL1_PROTOCOL': 2,      # MAVLink2
    'SERIAL1_BAUD': 115200,     # ✅ 修正: 115200 bps（正しい値）
    'BRD_SER1_RTSCTS': 0        # フロー制御無効
}

# すべてのパラメータを設定
for param_id, param_value in parameters.items():
    print(f"{param_id}を{param_value}に設定中...")
    
    # パラメータを設定
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        param_id.encode('utf-8'),
        param_value,
        mavutil.mavlink.MAV_PARAM_TYPE_INT32
    )
    
    # ACK（応答）を読み取る
    message = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=5)
    if message:
        message_dict = message.to_dict()
        # パラメータ名のデコード処理を追加
        param_name = message_dict['param_id'].decode('utf-8').rstrip('\x00')
        print(f"確認: {param_name} = {message_dict['param_value']}")
    else:
        print(f"警告: {param_id}の設定確認タイムアウト")
    
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

# EEPROM保存の確認
ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
if ack_msg:
    if ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("✅ パラメータがEEPROMに保存されました")
    else:
        print(f"⚠️ EEPROM保存結果: {ack_msg.result}")
else:
    print("⚠️ EEPROM保存の確認がタイムアウトしました")

print("すべてのパラメータが設定されました")

# 接続を閉じる
master.close()
