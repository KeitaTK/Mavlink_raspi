from pymavlink import mavutil
import time

# 接続を確立（実際の接続方法に合わせて変更してください）
# 例: シリアル接続 ('/dev/ttyACM0', baud=115200)
# 例: UDP接続 ('udpin:0.0.0.0:14550')  USB接続: 'COM3'など
# master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)

# ハートビートを待機（接続確認）
print("接続を待機中...")
master.wait_heartbeat()
print(f"接続完了 (システム: {master.target_system}, コンポーネント: {master.target_component})")

# DShot300に設定するパラメータ
params_to_set = {
    # # --- EKF3基本設定 ---
    # 'AHRS_EKF_TYPE': 3.0,
    # 'EK3_ENABLE': 1.0,
    # 'EK3_IMU_MASK': 1.0,
    
    # # --- GPS使用設定（設定A採用） ---
    # 'EK3_SRC1_POSXY': 3,     # GPS (水平位置)
    # 'EK3_SRC1_VELXY': 3,     # GPS (水平速度)
    # 'EK3_SRC1_POSZ': 3,      # GPS (垂直位置) 
    # 'EK3_SRC1_VELZ': 3,      # GPS (垂直速度)
    # 'EK3_SRC1_YAW': 2,       # GPS (ヨー角)
    
    # # --- EKF3精度設定 ---
    # 'EK3_GPS_CHECK': 1,      # 最小限チェック
    # 'EK3_POS_I_GATE': 5.0,   # 位置ゲート（緩和）
    # 'EK3_VEL_I_GATE': 8.0,   # 速度ゲート（緩和）
    # 'EK3_HGT_I_GATE': 5.0,   # 高度ゲート（緩和）
    # 'EK3_POSNE_M_NSE': 0.01, # 水平位置ノイズ: 1cm
    # 'EK3_VELNE_M_NSE': 0.01, # 水平速度ノイズ: 1cm/s
    # 'EK3_VELD_M_NSE': 0.01,  # 垂直速度ノイズ: 1cm/s
    # 'EK3_YAW_M_NSE': 0.1,    # ヨー角ノイズ
    # 'EK3_ALT_M_NSE': 100.0,  # 気圧センサーノイズ極大
    # 'EK3_ACC_P_NSE': 1.0,    # 加速度計ノイズ増大
    # 'EK3_PRIMARY': 0,
    
    # # --- GPS設定 ---
    # 'GPS1_TYPE': 14,         # MAVLink GPS Input
    # 'GPS_AUTO_CONFIG': 0,    # 自動設定無効
    # 'GPS_PRIMARY': 0,
    
    # # --- 超低速設定（設定B採用） ---
    # 'WPNAV_SPEED_UP': 10,    # 超低速上昇: 0.1m/s
    # 'WPNAV_SPEED_DN': 150,   # 下降速度: 1.5m/s
    # 'WPNAV_ACCEL_Z': 10,     # 超低加速度: 0.1m/s²
    # 'WPNAV_SPEED': 30,       # 水平速度: 0.3m/s
    # 'WPNAV_RADIUS': 20,      # 到達半径: 20cm（ゆっくり動作）
    
    # # --- パイロット制御速度 ---
    # 'PILOT_SPEED_UP': 250,   # パイロット上昇: 2.5m/s
    # 'PILOT_SPEED_DN': 150,   # パイロット下降: 1.5m/s
    # 'PILOT_ACCEL_Z': 250,    # パイロット加速度: 2.5m/s²
    
    # # --- 低ゲイン安定性重視PID（設定B採用） ---
    # 'PSC_POSZ_P': 2.0,       # 高度位置制御P（低ゲイン）
    # 'PSC_VELZ_P': 2.0,       # 垂直速度制御P（穏やか）
    # 'PSC_VELZ_I': 15.0,      # 垂直速度制御I
    # 'PSC_VELZ_D': 0.02,      # 垂直速度制御D
    # 'PSC_ACCZ_P': 0.15,      # 垂直加速度制御P（穏やか）
    # 'PSC_ACCZ_I': 0.3,       # 垂直加速度制御I（穏やか）
    
    # # --- 水平制御PID ---
    # 'PSC_POSXY_P': 2.0,      # 水平位置制御P
    # 'PSC_VELXY_P': 1.5,      # 水平速度制御P
    # 'PSC_VELXY_I': 3.0,      # 水平速度制御I
    
    # # --- 姿勢制御PID（低ゲイン安定性重視） ---
    # 'ATC_RAT_RLL_P': 0.05,   # Roll P（低ゲイン）
    # 'ATC_RAT_RLL_I': 0.05,   # Roll I
    # 'ATC_RAT_RLL_D': 0.001,  # Roll D
    # 'ATC_RAT_PIT_P': 0.05,   # Pitch P（低ゲイン）
    # 'ATC_RAT_PIT_I': 0.05,   # Pitch I
    # 'ATC_RAT_PIT_D': 0.001,  # Pitch D
    # 'ATC_RAT_YAW_P': 0.1,    # Yaw P
    # 'ATC_RAT_YAW_I': 0.01,   # Yaw I
    
    # # --- IMUフィルタ ---
    # 'INS_GYRO_FILTER': 20,   # ジャイロフィルタ
    # 'INS_ACCEL_FILTER': 20,  # 加速度フィルタ
    
    # # --- シリアル設定 ---
    # 'SERIAL1_PROTOCOL': 2,   # MAVLink2
    # 'SERIAL1_BAUD': 115200,  # 115200 bps
    # 'BRD_SER1_RTSCTS': 0,    # フロー制御無効
    # 'SERIAL2_PROTOCOL': 23,  # 追加設定
    
    # # --- RC設定 ---
    # 'RC10_OPTION': 56,       # Loiterモードスイッチ
    # 'RC11_OPTION': 55,       # GUIDEDモードスイッチ
    # 'THR_DZ': 200,          # スロットルデッドバンド
    # 'RC_OPTIONS': 10336,     # RC設定
    # 'RSSI_TYPE': 3,         # RSSI設定
    # 'RC9_OPTION': 153,       # RC9設定
    
    # # --- Loiterモード設定 ---
    # 'LOIT_SPEED': 50,        # Loiter速度: 0.5m/s
    # 'LOIT_ACC_MAX': 50,      # 最大加速度
    # 'LOIT_BRK_ACCEL': 50,    # ブレーキ加速度
    # 'LOIT_BRK_DELAY': 0.3,   # ブレーキ遅延
    # 'LOIT_BRK_JERK': 300,    # ブレーキ変化率
    # 'LOIT_ANG_MAX': 10,      # 最大傾斜角
    
    # # --- GUIDEDモード設定 ---
    # 'GUID_TIMEOUT': 3,       # タイムアウト: 3秒
    # 'GUID_OPTIONS': 0,       # デフォルトオプション
    
    # # --- コンパス無効化 ---
    # 'COMPASS_ENABLE': 0,     # コンパス無効
    # 'COMPASS_USE': 0.0,      # 内蔵コンパス無効
    # 'COMPASS_USE2': 0.0,     # 外付コンパス2無効  
    # 'COMPASS_USE3': 0.0,     # 外付コンパス3無効
    
    # # --- フェイルセーフ設定（AltHoldモード採用） ---
    # 'FS_EKF_ACTION': 3,      # EKF失敗時AltHoldモード
    # 'FS_EKF_THRESH': 0.8,    # EKF信頼度閾値
    # 'FS_THR_ENABLE': 3,      # 送信機喪失時着陸
    # 'FS_THR_VALUE': 975,     # 失効検出値
    # 'FS_OPTIONS': 32,        # 即時終了オプション
    # 'FS_CRASH_CHECK': 1,     # クラッシュ検出有効
    # 'FS_VIBE_ENABLE': 1,     # 振動検出有効
    # 'FS_DR_ENABLE': 1,       # Dead Reckoning有効
    # 'FS_DR_TIMEOUT': 15,     # 15秒後フェイルセーフ
    # 'RTL_ALT': 50,          # RTL高度: 50cm
    
    # # --- モーター出力設定（DShot） ---
    # 'SERVO1_FUNCTION': 0,    # MAIN無効化
    # 'SERVO2_FUNCTION': 0,    # MAIN無効化
    # 'SERVO3_FUNCTION': 0,    # MAIN無効化
    # 'SERVO4_FUNCTION': 0,    # MAIN無効化
    # 'SERVO9_FUNCTION': 33,   # AUXをモーター1
    # 'SERVO10_FUNCTION': 34,  # AUXをモーター2
    # 'SERVO11_FUNCTION': 35,  # AUXをモーター3
    # 'SERVO12_FUNCTION': 36,  # AUXをモーター4
    # 'MOT_PWM_TYPE': 4,       # DShot300
    # 'SERVO_DSHOT_ESC': 2,    # BLHeli_S/BlueJay
    # 'SERVO_BLH_MASK': 3840,  # チャンネル9-12
    # 'SERVO_BLH_AUTO': 1,     # パススルー有効
    
    # # --- 安全設定 ---
    # 'BRD_SAFETY_DEFLT': 0,   # Safety switch無効
    # 'BRD_SAFETYOPTION': 0,   # 安全オプション
    # 'ARMING_CHECK': 80,      # アーミングチェック
    # 'ARMING_RUDDER': 0,      # ラダーアーミング無効
    # 'DISARM_DELAY': 0,       # 即時ディスアーム
    # 'MOT_SPIN_ARM': 0.02,    # アーム時モーター回転
    # 'MOT_SPIN_MIN': 0.02,    # 最小モーター回転
    
    # # --- バッテリー設定（5セルLiIon） ---
    # 'BATT_MONITOR': 3,       # 電圧監視
    # 'BATT_ARM_VOLT': 16.0,   # アーム可能電圧
    # 'BATT_CRT_VOLT': 14.0,   # 危険電圧
    # 'BATT_LOW_VOLT': 15.5,   # 警告電圧
    # 'BATT_CAPACITY': 0,      # 容量監視無効
    # 'BATT_ARM_MAH': 0,       # 容量アーミング無効
    # 'BATT_CRT_MAH': 0,       # 危険容量無効
    # 'BATT_LOW_MAH': 0,       # 警告容量無効
    # 'MOT_BAT_VOLT_MAX': 21.0, # 最大電圧
    # 'MOT_BAT_VOLT_MIN': 13.5, # 最小電圧


    # --- EKF3基本設定 ---
    'AHRS_EKF_TYPE': 3.0,
    'EK3_ENABLE': 1.0,
    'EK3_IMU_MASK': 1.0,
    
    # --- GPS使用設定 ---
    'EK3_SRC1_POSXY': 3,     # GPS (水平位置)
    'EK3_SRC1_VELXY': 3,     # GPS (水平速度)
    'EK3_SRC1_POSZ': 3,      # GPS (垂直位置) 
    'EK3_SRC1_VELZ': 3,      # GPS (垂直速度)
    'EK3_SRC1_YAW': 2,       # GPS (ヨー角)
    
    # --- EKF3精度設定（現実的な値に修正） ---
    'EK3_GPS_CHECK': 0,      # GPS健全性チェック完全無効化
    'EK3_POS_I_GATE': 15.0,  # 位置ゲート大幅緩和（5→15）
    'EK3_VEL_I_GATE': 15.0,  # 速度ゲート大幅緩和（8→15）
    'EK3_HGT_I_GATE': 15.0,  # 高度ゲート大幅緩和（5→15）
    
    # --- ノイズパラメータ（現実的な値）---
    'EK3_POSNE_M_NSE': 0.5,  # 水平位置ノイズ: 50cm（0.01→0.5、50倍緩和）[2]
    'EK3_VELNE_M_NSE': 0.5,  # 水平速度ノイズ: 50cm/s（0.01→0.5）[7]
    'EK3_VELD_M_NSE': 0.5,   # 垂直速度ノイズ: 50cm/s（0.01→0.5）[7]
    'EK3_YAW_M_NSE': 1.0,    # ヨー角ノイズ緩和（0.1→1.0）
    
    # --- センサーノイズ調整 ---
    'EK3_ALT_M_NSE': 10.0,   # 気圧センサーノイズ（100→10、適度に使用）[2]
    'EK3_ACC_P_NSE': 0.5,    # 加速度計ノイズ削減（1.0→0.5）
    'EK3_GYR_P_NSE': 0.03,   # ジャイロノイズ（デフォルト値）
    
    # --- EKF安定化追加設定 ---
    'EK3_GLITCH_RAD': 25,    # GPS Glitch検出半径緩和[6]
    'EK3_CHECK_SCALE': 200,  # EKFチェックスケール緩和
    'EK3_PRIMARY': 0,
    
    # --- GPS設定 ---
    'GPS1_TYPE': 14,         # MAVLink GPS Input
    'GPS_AUTO_CONFIG': 0,    # 自動設定無効
    'GPS_PRIMARY': 0,
    
    # --- 超低速設定（EKF負荷軽減）---
    'WPNAV_SPEED_UP': 20,    # 上昇速度緩和（10→20、0.2m/s）
    'WPNAV_SPEED_DN': 100,   # 下降速度緩和（150→100）
    'WPNAV_ACCEL_Z': 25,     # 加速度緩和（10→25、0.25m/s²）
    'WPNAV_SPEED': 50,       # 水平速度緩和（30→50、0.5m/s）
    'WPNAV_RADIUS': 30,      # 到達半径緩和（20→30cm）
    
    # --- パイロット制御速度 ---
    'PILOT_SPEED_UP': 250,   # パイロット上昇: 2.5m/s
    'PILOT_SPEED_DN': 150,   # パイロット下降: 1.5m/s
    'PILOT_ACCEL_Z': 250,    # パイロット加速度: 2.5m/s²
    
    # --- 安定化PID（さらに保守的に） ---
    'PSC_POSZ_P': 1.0,       # 高度位置制御P（2.0→1.0）
    'PSC_VELZ_P': 3.0,       # 垂直速度制御P（2.0→3.0、適度に上げる）
    'PSC_VELZ_I': 6.0,       # 垂直速度制御I（15.0→6.0、下げる）
    'PSC_VELZ_D': 0.01,      # 垂直速度制御D（0.02→0.01）
    'PSC_ACCZ_P': 0.3,       # 垂直加速度制御P（0.15→0.3）
    'PSC_ACCZ_I': 1.0,       # 垂直加速度制御I（0.3→1.0）
    
    # --- 水平制御PID ---
    'PSC_POSXY_P': 1.5,      # 水平位置制御P（2.0→1.5）
    'PSC_VELXY_P': 1.0,      # 水平速度制御P（1.5→1.0）
    'PSC_VELXY_I': 2.0,      # 水平速度制御I（3.0→2.0）
    
    # --- 姿勢制御PID（安定性重視維持） ---
    'ATC_RAT_RLL_P': 0.08,   # Roll P（0.05→0.08、少し上げる）
    'ATC_RAT_RLL_I': 0.08,   # Roll I
    'ATC_RAT_RLL_D': 0.002,  # Roll D
    'ATC_RAT_PIT_P': 0.08,   # Pitch P（0.05→0.08）
    'ATC_RAT_PIT_I': 0.08,   # Pitch I
    'ATC_RAT_PIT_D': 0.002,  # Pitch D
    'ATC_RAT_YAW_P': 0.15,   # Yaw P（0.1→0.15）
    'ATC_RAT_YAW_I': 0.02,   # Yaw I（0.01→0.02）
    
    # --- IMUフィルタ（ノイズ削減） ---
    'INS_GYRO_FILTER': 40,   # ジャイロフィルタ強化（20→40）
    'INS_ACCEL_FILTER': 40,  # 加速度フィルタ強化（20→40）
    
    # --- フェイルセーフ設定（緩和） ---
    'FS_EKF_ACTION': 2,      # EKF失敗時Stabilizeモード（3→2）[4]
    'FS_EKF_THRESH': 1.0,    # EKF信頼度閾値最大緩和（0.8→1.0）[4]
    'FS_THR_ENABLE': 3,      # 送信機喪失時着陸
    'FS_THR_VALUE': 975,     # 失効検出値
    'FS_OPTIONS': 0,         # フェイルセーフオプション緩和（32→0）
    'FS_CRASH_CHECK': 0,     # クラッシュ検出無効化（1→0）
    'FS_VIBE_ENABLE': 0,     # 振動検出無効化（1→0）
    'FS_DR_ENABLE': 0,       # Dead Reckoning無効化（1→0）
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
    
    'COMPASS_ENABLE': 0,
    'COMPASS_USE': 0.0,
    'COMPASS_USE2': 0.0,
    'COMPASS_USE3': 0.0,
    
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
ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=10)
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
    time.sleep(0.1)

print("設定と保存の確認完了")
