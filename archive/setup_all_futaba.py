from pymavlink import mavutil
import time

# master = mavutil.mavlink_connection('/dev/ttyAMA0', 921600, rtscts=True)  # フロー制御
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)  # USB接続

# ハートビートを待機（接続確認）
print("接続を待機中...")
master.wait_heartbeat()
print(f"接続完了 (システム: {master.target_system}, コンポーネント: {master.target_component})")

params_to_set = {
    # --- EKF3基本設定 ---
    'AHRS_EKF_TYPE': 3.0,
    'EK3_ENABLE': 1.0,
    'EK3_IMU_MASK': 3,
    
    # --- ハイブリッド設定：GPS位置+コンパスヨー角（GPS補助）、速度はIMU推定 ---
    'EK3_SRC1_POSXY': 3,     # GPS (水平位置)
    'EK3_SRC1_VELXY': 0,     # None (水平速度をIMUから推定)
    'EK3_SRC1_POSZ': 3,      # GPS (垂直位置)
    'EK3_SRC1_VELZ': 0,      # None (垂直速度をIMUから推定)
    'EK3_SRC1_YAW': 3,       # GPS with compass fallback（ハイブリッド）[1]
    
    # --- EKF3精度設定（適正化） ---
    'EK3_GPS_CHECK': 0,      # GPS健全性チェック完全無効化
    'EK3_POS_I_GATE': 8.0,  # 位置ゲート(デフォルト5)
    'EK3_VEL_I_GATE': 8.0,   # 速度ゲート（IMU推定精度向上）
    'EK3_HGT_I_GATE': 10.0,  # 高度ゲート
    
    # --- ノイズパラメータ（ハイブリッド用調整） --- これを小さくするとGPS情報をより信用
    'EK3_POSNE_M_NSE': 0.3,  # 水平位置ノイズ:
    'EK3_VELNE_M_NSE': 0.5,  # 水平速度ノイズ: 50cm/s
    'EK3_VELD_M_NSE': 0.5,   # 垂直速度ノイズ: 50cm/s
    'EK3_YAW_M_NSE': 0.2,    # ヨー角ノイズ（ハイブリッド用に緩和）
    
    # --- センサーノイズ調整 ---
    'EK3_ALT_M_NSE': 10.0,   # 気圧センサーノイズ
    'EK3_GYRO_P_NSE': 0.02,  # ジャイロプロセスノイズ
    
    # --- コンパス設定（ハイブリッド用有効化） ---
    'COMPASS_ENABLE': 1,     # コンパス有効化[1]
    'COMPASS_USE': 1.0,      # 内蔵コンパス使用[1]
    'COMPASS_USE2': 0.0,     # 外付コンパス2無効
    'COMPASS_USE3': 0.0,     # 外付コンパス3無効
    'COMPASS_AUTODEC': 1,    # 自動磁気偏角有効[1]
    'COMPASS_LEARN': 1,      # コンパス学習有効[1]
    
    # --- ハイブリッド設定（重要）[4] ---
    'EK3_MAG_CAL': 3,        # 地上でheading fusion、空中で3-axis fusion[4]
    'EK3_SRC_OPTIONS': 1,    # Fuse all velocity sources[1]
    
    # --- EKF安定化設定（適正化） ---
    'EK3_GLITCH_RAD': 5,    # GPS Glitch検出半径緩和
    'EK3_CHECK_SCALE': 100,  # EKFチェックスケール（200→100に適正化）
    'EK3_PRIMARY': -1, # 自動切り替え無効
    
    # --- GPS設定 ---
    'GPS1_TYPE': 14,         # MAVLink GPS Input
    'GPS_AUTO_CONFIG': 0,    # 自動設定無効
    'GPS_PRIMARY': 0,
    
    # --- Guidedモード設定 ---
    'WPNAV_SPEED_UP': 30,    # 上昇速度: 0.2m/s
    'WPNAV_SPEED_DN': 20,   # 下降速度: 1.0m/s
    'WPNAV_ACCEL_Z': 30,     # 加速度: 0.25m/s²
    'WPNAV_SPEED': 150,       # 水平速度:
    'WPNAV_ACCEL':  250,    # 水平加速度
    'WPNAV_RADIUS': 20,      # 到達半径: 30cm

    # --- Loiterモード設定 ---
    'LOIT_SPEED': 50,
    'LOIT_ACC_MAX': 50,
    'LOIT_BRK_ACCEL': 50,
    'LOIT_BRK_DELAY': 0.3,
    'LOIT_BRK_JERK': 300,
    'LOIT_ANG_MAX': 10,
    
    # --- パイロット制御速度 ---
    'PILOT_SPEED_UP': 250,   # パイロット上昇: 2.5m/s
    'PILOT_SPEED_DN': 150,   # パイロット下降: 1.5m/s
    'PILOT_ACCEL_Z': 250,    # パイロット加速度: 2.5m/s²
    
    # --- 垂直制御PID ---
    'PSC_POSZ_P': 1.0,       # 高度位置制御P
    'PSC_VELZ_P': 4.0,       # 垂直速度制御P
    'PSC_VELZ_I': 8.0,       # 垂直速度制御I
    'PSC_VELZ_D': 0.01,      # 垂直速度制御D
    'PSC_ACCZ_P': 0.3,       # 垂直加速度制御P
    'PSC_ACCZ_I': 1.0,       # 垂直加速度制御I
    
    # --- 水平制御PID ---
    'PSC_POSXY_P': 1.5,      # 水平位置制御P（1.0→1.5に増加）
    'PSC_VELXY_P': 1.1,      # 水平速度制御P（1.2→2.0に増加）
    'PSC_VELXY_I': 0.2,      # 水平速度制御I（2.5→1.0に削減）
    'PSC_VELXY_D': 0.25,      # 水平速度制御D（追加）
    
    # --- 姿勢制御PID ---
    'ATC_RAT_RLL_P': 0.03,  # Roll P（0.05→0.135、標準値）
    'ATC_RAT_RLL_I': 0.05,  # Roll I（0.05→0.135、標準値）
    'ATC_RAT_RLL_D': 0.002, # Roll D（0.001→0.0036、標準値）
    'ATC_RAT_PIT_P': 0.04,  # Pitch P（0.05→0.135、標準値）
    'ATC_RAT_PIT_I': 0.05,  # Pitch I（0.05→0.135、標準値）
    'ATC_RAT_PIT_D': 0.002, # Pitch D（0.001→0.0036、標準値）
    'ATC_RAT_YAW_P': 0.2,    # Yaw P（ハイブリッドヨー対応）
    'ATC_RAT_YAW_I': 0.02,   # Yaw I
    
    # --- IMUフィルタ（応答性向上） ---
    'INS_GYRO_FILTER': 20,   # ジャイロフィルタ（30→20、応答性向上）
    'INS_ACCEL_FILTER': 20,  # 加速度フィルタ（30→20、応答性向上）
    
    # --- フェイルセーフ設定（適正化） ---
    'FS_EKF_ACTION': 2,      # EKF失敗時Stabilizeモード
    'FS_EKF_THRESH': 0.8,    # EKF信頼度閾値（1.0→0.8に適正化）
    'FS_THR_ENABLE': 3,      # 送信機喪失時着陸
    'FS_THR_VALUE': 975,     # 失効検出値
    'FS_OPTIONS': 0,         # フェイルセーフオプション緩和
    'FS_CRASH_CHECK': 0,     # クラッシュ検出無効化
    'FS_VIBE_ENABLE': 0,     # 振動検出無効化
    'FS_DR_ENABLE': 0,       # Dead Reckoning無効化
    'RTL_ALT': 50,          # RTL高度: 50cm
    
    # --- シリアル設定 ---
    'SERIAL1_PROTOCOL': 2,
    'SERIAL1_BAUD': 921600,
    'BRD_SER1_RTSCTS': 2,    # ハードウェアフロー制御有効（0→2に変更）
    # 'SERIAL2_PROTOCOL': 23,  # ELRSレシーバー
    'SERIAL2_PROTOCOL': 2,  # 双葉レシーバー
    
    # --- RC設定 ---
    'RC6_OPTION': 56,  # Loiter
    'RC7_OPTION': 55,  # Guided
    'THR_DZ': 200,
    'RC_OPTIONS': 10336,
    # 'RSSI_TYPE': 3,  # ELRSレシーバー
    'RSSI_TYPE': 0,  # 双葉レシーバー
    'RC8_OPTION': 153,  # アーム設定
    
    # --- GUIDEDモード設定 ---
    'GUID_TIMEOUT': 3,
    'GUID_OPTIONS': 0,

    # --- ログ制御設定（アーム時のみ記録、ファイルローテーション） ---
    'LOG_DISARMED': 0,        # 非アーム時はログを記録しない
    'LOG_FILE_DSRMROT': 1,    # ディスアーム時にログファイルをローテーション
    'LOG_FILE_TIMEOUT': 5,    # ログファイルのタイムアウト（秒）
    'LOG_BACKEND_TYPE': 1,    # デフォルトのログバックエンド（ファイル）
    
    # --- モーター・安全設定 ---
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
    
    # --- バッテリー設定 ---
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
    # time.sleep(0.1)

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
ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
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
    # time.sleep(0.1)

print("設定と保存の確認完了")
