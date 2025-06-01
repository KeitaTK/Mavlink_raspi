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
    # 基本EKF設定
    'AHRS_EKF_TYPE': 3,        # EKF3を使用
    'EK2_ENABLE': 0,           # EKF2を無効
    'EK3_ENABLE': 1,           # EKF3を有効
    'EK3_IMU_MASK': 1.0,    # シングルIMU使用
    'EK3_GPS_TYPE': 0.0,    # GPS無効
    'EK3_ALT_SOURCE': 0.0,   # デフォルト高度ソース
    
    # 外部位置データ用設定（モーションキャプチャ）
    'EK3_SRC1_POSXY': 6,       # ExternalNav（水平位置）
    'EK3_SRC1_POSZ': 6,        # ExternalNav（垂直位置）
    'EK3_SRC1_VELXY': 0,       # None（速度は位置から推定）
    'EK3_SRC1_VELZ': 0,        # None
    'EK3_SRC1_YAW': 6,         # ExternalNav（ヨー角）
    
    # イノベーションゲート調整
    'EK3_POS_I_GATE': 3,       # 位置イノベーションゲート
    'EK3_VEL_I_GATE': 5,       # 速度イノベーションゲート
    'EK3_HGT_I_GATE': 3,       # 高度イノベーションゲート

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
