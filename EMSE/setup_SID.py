from pymavlink import mavutil
import time

if input("USB接続を行いますか (y/n): ").strip().lower() == 'y':
    # USB接続の場合
    master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)  # USB接続
else:
    # UART接続の場合
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=1000000, rtscts=True) # フロー制御

# ハートビートを待機（接続確認）
print("接続を待機中...")
master.wait_heartbeat()
print(f"接続完了 (システム: {master.target_system}, コンポーネント: {master.target_component})")

params_to_set = {
    # --- EKF3基本設定 ---
    'AHRS_EKF_TYPE': 3.0,
    'EK3_ENABLE': 1.0,
    'EK3_IMU_MASK': 3,
    # --- ハイブリッド設定 ---
    'EK3_SRC1_POSXY': 3,
    'EK3_SRC1_VELXY': 3,
    'EK3_SRC1_POSZ': 3,
    'EK3_SRC1_VELZ': 3,
    'EK3_SRC1_YAW': 3,
    # --- EKF3精度設定 ---
    'EK3_GPS_CHECK': 1,
    'EK3_POS_I_GATE': 8.0,
    'EK3_VEL_I_GATE': 8.0,
    'EK3_HGT_I_GATE': 10.0,
    # --- ノイズ調整 ---
    'EK3_POSNE_M_NSE': 0.2,
    'EK3_VELNE_M_NSE': 0.3,
    'EK3_VELD_M_NSE': 0.5,
    'EK3_YAW_M_NSE': 0.2,
    # --- センサーノイズ ---
    'EK3_ALT_M_NSE': 10.0,
    'EK3_GYRO_P_NSE': 0.02,
    # --- コンパス設定 ---
    'COMPASS_ENABLE': 1,
    'COMPASS_USE': 1.0,
    'COMPASS_USE2': 0.0,
    'COMPASS_USE3': 0.0,
    'COMPASS_AUTODEC': 1,
    'COMPASS_LEARN': 1,
    # --- ハイブリッド設定（重要） ---
    'EK3_MAG_CAL': 3,
    'EK3_SRC_OPTIONS': 1,
    # --- EKF安定化 ---
    'EK3_GLITCH_RAD': 5,
    'EK3_CHECK_SCALE': 100,
    'EK3_PRIMARY': -1,
    # --- GPS設定 ---
    'GPS1_TYPE': 14,
    'GPS_AUTO_CONFIG': 0,
    'GPS_PRIMARY': 0,
    # --- システムIDモード ---
    'SID_AXIS': 10,

    # --- フライトモード切替チャンネル ---
    'FLTMODE_CH': 5,     # Channel 5 をフライトモードスイッチに

    # --- Channel5 の 6ポジション割り当て ---
    'FLTMODE1': 5,       # ポジション1 = Loiter
    'FLTMODE2': 16,      # ポジション2 = Guided
    'FLTMODE3': 25,      # ポジション3 = SystemID
    'FLTMODE4': 0,       # ポジション4 = Stabilize (フォールバック)
    'FLTMODE5': 0,       # ポジション5 = Stabilize (フォールバック)
    'FLTMODE6': 0,       # ポジション6 = Stabilize (フォールバック)

    # --- その他の設定はそのまま保持 ---
    'WPNAV_SPEED_UP': 40,
    'WPNAV_SPEED_DN': 30,
    'WPNAV_ACCEL_Z': 70,
    'WPNAV_SPEED': 500,
    'WPNAV_ACCEL': 500,
    'WPNAV_RADIUS': 5,
    'LOIT_SPEED': 50,
    'LOIT_ACC_MAX': 50,
    'LOIT_BRK_ACCEL': 50,
    'LOIT_BRK_DELAY': 0.3,
    'LOIT_BRK_JERK': 300,
    'LOIT_ANG_MAX': 10,
    'PILOT_SPEED_UP': 250,
    'PILOT_SPEED_DN': 150,
    'PILOT_ACCEL_Z': 250,
    'PSC_POSZ_P': 1,
    'PSC_VELZ_P': 4.0,
    'PSC_VELZ_I': 8.0,
    'PSC_VELZ_D': 0.01,
    'PSC_ACCZ_P': 0.3,
    'PSC_ACCZ_I': 1.0,
    'PSC_POSXY_P': 5.0,
    'PSC_VELXY_P': 3,
    'PSC_VELXY_I': 2.5,
    'PSC_VELXY_D': 0.5,
    'ATC_ANG_RLL_P': 4.5,
    'ATC_ANG_PIT_P': 4.5,
    'ATC_ANG_YAW_P': 4.5,
    'ATC_RAT_RLL_P': 0.04,
    'ATC_RAT_RLL_I': 0.0,
    'ATC_RAT_RLL_D': 0.0012,
    'ATC_RAT_RLL_FF': 0.0,
    'ATC_RAT_PIT_P': 0.05,
    'ATC_RAT_PIT_I': 0.0,
    'ATC_RAT_PIT_D': 0.0012,
    'ATC_RAT_PIT_FF': 0.0,
    'ATC_RAT_YAW_P': 0.2,
    'ATC_RAT_YAW_I': 0.0,
    'ATC_RAT_YAW_FF': 0.0,
    'INS_GYRO_FILTER': 20,
    'INS_ACCEL_FILTER': 20,
    'FS_EKF_ACTION': 2,
    'FS_EKF_THRESH': 0.8,
    'FS_THR_ENABLE': 3,
    'FS_THR_VALUE': 975,
    'FS_OPTIONS': 0,
    'FS_CRASH_CHECK': 0,
    'FS_VIBE_ENABLE': 0,
    'FS_DR_ENABLE': 0,
    'RTL_ALT': 50,
    'SERIAL1_PROTOCOL': 2,
    'SERIAL1_BAUD': 1000000,
    'BRD_SER1_RTSCTS': 2,
    'SERIAL2_PROTOCOL': 23,
    'RC10_OPTION': 56,  # Guided 用 AUX10 設定
    'RC11_OPTION': 55,  # Loiter 用 AUX11 設定
    'THR_DZ': 200,
    'RC_OPTIONS': 10336,
    'RSSI_TYPE': 3,
    'RC9_OPTION': 153,
    'GUID_TIMEOUT': 3,
    'GUID_OPTIONS': 0,
    'LOG_DISARMED': 0,
    'LOG_FILE_DSRMROT': 1,
    'LOG_FILE_TIMEOUT': 5,
    'LOG_BACKEND_TYPE': 1,
    'LOG_BITMASK': 176126,
    'MOT_THST_HOVER': 0.223,
    'MOT_THST_EXPO': 0,
    'MOT_HOVER_LEARN': 0,
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
    if param_name == 'OBS_CORR_GAIN':
        # 赤色で強調表示
        print(f'\033[91mパラメータ設定: {param_name} = {message["param_value"]}  # 吊荷補正ゲイン\033[0m')
    else:
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
ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=1)
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
    
    message = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=10)
    if message:
        message = message.to_dict()
        print(f'確認: {param_name} = {message["param_value"]}')
    else:
        print(f'確認: {param_name} のデータを取得できませんでした')
    # time.sleep(0.1)

print("設定と保存の確認完了")
