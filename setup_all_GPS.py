from pymavlink import mavutil
import time

# 接続を確立（実際の接続方法に合わせて変更してください）
# 例: シリアル接続 ('/dev/ttyACM0', baud=115200)
# 例: UDP接続 ('udpin:0.0.0.0:14550')  USB接続: 'COM3'など
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
# master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)

# ハートビートを待機（接続確認）
print("接続を待機中...")
master.wait_heartbeat()
print(f"接続完了 (システム: {master.target_system}, コンポーネント: {master.target_component})")

# DShot300に設定するパラメータ
params_to_set = {
    
    # --- EKF3基本設定 ---
    'AHRS_EKF_TYPE': 3.0,
    'EK3_ENABLE': 1.0,
    'EK3_IMU_MASK': 1.0,
    
    # --- GPS使用に変更：GPS位置+GPSヨー角、速度はIMU推定 ---
    'EK3_SRC1_POSXY': 3,     # GPS (水平位置) - 維持
    'EK3_SRC1_VELXY': 0,     # None (水平速度をIMUから推定) - 維持
    'EK3_SRC1_POSZ': 3,      # GPS (垂直位置) - 維持
    'EK3_SRC1_VELZ': 0,      # None (垂直速度をIMUから推定) - 維持
    'EK3_SRC1_YAW': 2,       # GPS (ヨー角) ← Compass→GPSに変更[1][3]
    
    # --- EKF3精度設定 ---
    'EK3_GPS_CHECK': 0,      # GPS健全性チェック完全無効化
    'EK3_POS_I_GATE': 15.0,  # 位置ゲート大幅緩和
    'EK3_VEL_I_GATE': 8.0,   # 速度ゲート（IMU推定精度向上）
    'EK3_HGT_I_GATE': 15.0,  # 高度ゲート大幅緩和
    
    # --- ノイズパラメータ（現状維持） ---
    'EK3_POSNE_M_NSE': 0.5,  # 水平位置ノイズ: 50cm
    'EK3_VELNE_M_NSE': 0.5,  # 水平速度ノイズ: 50cm/s
    'EK3_VELD_M_NSE': 0.5,   # 垂直速度ノイズ: 50cm/s
    'EK3_YAW_M_NSE': 1.0,    # ヨー角ノイズ（GPS用）
    
    # --- センサーノイズ調整 ---
    'EK3_ALT_M_NSE': 10.0,   # 気圧センサーノイズ
    'EK3_GYRO_P_NSE': 0.02,  # ジャイロプロセスノイズ
    
    # --- コンパス設定（無効化に戻す） ---
    'COMPASS_ENABLE': 0,     # コンパス無効化（1→0）[3]
    'COMPASS_USE': 0.0,      # 内蔵コンパス無効（1→0）
    'COMPASS_USE2': 0.0,     # 外付コンパス2無効
    'COMPASS_USE3': 0.0,     # 外付コンパス3無効
    
    # --- GPS ヨー角設定（検索結果より） ---
    'EK3_MAG_CAL': 3,        # GPS ヨー使用時の磁気校正設定[4]
    'EK3_SRC_OPTIONS': 0,    # Disable FuseAllVelocities[2]
    
    # --- EKF安定化追加設定 ---
    'EK3_GLITCH_RAD': 25,    # GPS Glitch検出半径緩和
    'EK3_CHECK_SCALE': 200,  # EKFチェックスケール緩和
    'EK3_PRIMARY': 0,
    
    # --- GPS設定 ---
    'GPS1_TYPE': 14,         # MAVLink GPS Input
    'GPS_AUTO_CONFIG': 0,    # 自動設定無効
    'GPS_PRIMARY': 0,
    
    # --- 超低速設定 ---
    'WPNAV_SPEED_UP': 20,    # 上昇速度: 0.2m/s
    'WPNAV_SPEED_DN': 100,   # 下降速度: 1.0m/s
    'WPNAV_ACCEL_Z': 25,     # 加速度: 0.25m/s²
    'WPNAV_SPEED': 50,       # 水平速度: 0.5m/s
    'WPNAV_RADIUS': 30,      # 到達半径: 30cm
    
    # --- パイロット制御速度 ---
    'PILOT_SPEED_UP': 250,   # パイロット上昇: 2.5m/s
    'PILOT_SPEED_DN': 150,   # パイロット下降: 1.5m/s
    'PILOT_ACCEL_Z': 250,    # パイロット加速度: 2.5m/s²
    
    # --- 安定化PID ---
    'PSC_POSZ_P': 1.0,       # 高度位置制御P
    'PSC_VELZ_P': 4.0,       # 垂直速度制御P（IMU推定対応）
    'PSC_VELZ_I': 8.0,       # 垂直速度制御I
    'PSC_VELZ_D': 0.01,      # 垂直速度制御D
    'PSC_ACCZ_P': 0.3,       # 垂直加速度制御P
    'PSC_ACCZ_I': 1.0,       # 垂直加速度制御I
    
    # --- 水平制御PID ---
    'PSC_POSXY_P': 1.5,      # 水平位置制御P
    'PSC_VELXY_P': 1.2,      # 水平速度制御P（IMU推定対応）
    'PSC_VELXY_I': 2.5,      # 水平速度制御I
    
    # --- 姿勢制御PID ---
    'ATC_RAT_RLL_P': 0.1,    # Roll P
    'ATC_RAT_RLL_I': 0.1,    # Roll I
    'ATC_RAT_RLL_D': 0.003,  # Roll D
    'ATC_RAT_PIT_P': 0.1,    # Pitch P
    'ATC_RAT_PIT_I': 0.1,    # Pitch I
    'ATC_RAT_PIT_D': 0.003,  # Pitch D
    'ATC_RAT_YAW_P': 0.2,    # Yaw P（GPS ヨー対応）
    'ATC_RAT_YAW_I': 0.02,   # Yaw I
    
    # --- IMUフィルタ ---
    'INS_GYRO_FILTER': 30,   # ジャイロフィルタ
    'INS_ACCEL_FILTER': 30,  # 加速度フィルタ
    
    # --- フェイルセーフ設定 ---
    'FS_EKF_ACTION': 2,      # EKF失敗時Stabilizeモード
    'FS_EKF_THRESH': 1.0,    # EKF信頼度閾値最大緩和
    'FS_THR_ENABLE': 3,      # 送信機喪失時着陸
    'FS_THR_VALUE': 975,     # 失効検出値
    'FS_OPTIONS': 0,         # フェイルセーフオプション緩和
    'FS_CRASH_CHECK': 0,     # クラッシュ検出無効化
    'FS_VIBE_ENABLE': 0,     # 振動検出無効化
    'FS_DR_ENABLE': 0,       # Dead Reckoning無効化
    'RTL_ALT': 50,          # RTL高度: 50cm
    
    # --- その他の設定（変更なし） ---
    'SERIAL1_PROTOCOL': 2,
    'SERIAL1_BAUD': 115200,
    'BRD_SER1_RTSCTS': 0,
    'SERIAL2_PROTOCOL': 23,
    
    'RC10_OPTION': 56,
    'RC11_OPTION': 55,
    'THR_DZ': 200,
    'RC_OPTIONS': 10336,
    'RSSI_TYPE': 3,
    'RC9_OPTION': 153,
    
    'LOIT_SPEED': 50,
    'LOIT_ACC_MAX': 50,
    'LOIT_BRK_ACCEL': 50,
    'LOIT_BRK_DELAY': 0.3,
    'LOIT_BRK_JERK': 300,
    'LOIT_ANG_MAX': 10,
    
    'GUID_TIMEOUT': 3,
    'GUID_OPTIONS': 0,
    
    # --- モーター・安全設定（変更なし） ---
    'SERVO1_FUNCTION': 0,
    'SERVO2_FUNCTION': 0,
    'SERVO3_FUNCTION': 0,
    'SERVO4_FUNCTION': 0,
    'SERVO9_FUNCTION': 33,
    'SERVO10_FUNCTION': 34,
    'SERVO11_FUNCTION': 35,
    'SERVO12_FUNCTION': 36,
    'MOT_PWM_TYPE': 4,
    'SERVO_DSHOT_ESC': 2,
    'SERVO_BLH_MASK': 3840,
    'SERVO_BLH_AUTO': 1,
    
    'BRD_SAFETY_DEFLT': 0,
    'BRD_SAFETYOPTION': 0,
    'ARMING_CHECK': 80,
    'ARMING_RUDDER': 0,
    'DISARM_DELAY': 0,
    'MOT_SPIN_ARM': 0.02,
    'MOT_SPIN_MIN': 0.02,
    
    'BATT_MONITOR': 3,
    'BATT_ARM_VOLT': 16.0,
    'BATT_CRT_VOLT': 14.0,
    'BATT_LOW_VOLT': 15.5,
    'BATT_CAPACITY': 0,
    'BATT_ARM_MAH': 0,
    'BATT_CRT_MAH': 0,
    'BATT_LOW_MAH': 0,
    'MOT_BAT_VOLT_MAX': 21.0,
    'MOT_BAT_VOLT_MIN': 13.5,

}




# パラメータを設定
print("パラメータを設定中...")
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
    time.sleep(0.1)

time.sleep(5)

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
ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=20)
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
    
    message = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=10).to_dict()
    if message:
        print(f'確認: {param_name} = {message["param_value"]}')
    else:
        print(f'確認: {param_name} のデータを取得できませんでした')
    time.sleep(0.1)

print("設定と保存の確認完了")
