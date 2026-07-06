#!/usr/bin/env python3
"""
ArduPilot GPS/RTK パラメータ設定 (Pixhawk6C)

GPS_INJECT_TO=0 で RTCM のシリアルポート注入を停止（CAN1経由のGPSへは影響なし）。
EKF3 ノイズパラメータと GPS 信頼度を適正化。

前提: sudo systemctl stop mavlink-router で UART を解放してから実行
"""

from pymavlink import mavutil
import time

# === 接続 ===
if input("USB接続 (/dev/ttyACM0) を使いますか (y/n): ").strip().lower() == 'y':
    master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
else:
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)

print("接続待機中...")
master.wait_heartbeat()
print(f"接続完了 (sys={master.target_system}, comp={master.target_component})")

# === パラメータ定義 ===
params_to_set = {
    # ── RTCM注入先: 127(全ポート)→0(無効) で TELEM1 への混入を防止 ──
    'GPS_INJECT_TO': (0, 'AP_Int8', 'RTCM注入先ポート (0=無効, 127=全ポート)'),

    # ── GPS 自動設定 ──
    'GPS_AUTO_CONFIG': (2, 'AP_Int8', '自動設定 (2=DroneCANのみ)'),
    'GPS_AUTO_SWITCH': (1, 'AP_Int8', 'GPS自動切替有効'),

    # ── GPS 測位信頼度 ──
    'GPS_HDOP_GOOD': (120, 'AP_Int16', '良好HDOP閾値 (×100, 120=1.2)'),
    'GPS_SBAS_MODE': (1, 'AP_Int8', 'SBAS補正有効'),
    'GPS_PRIMARY': (0, 'AP_Int8', 'プライマリGPS'),

    # ── EKF3: GPS ソース ──
    'AHRS_EKF_TYPE': (3, 'AP_Int8', 'EKF3使用'),
    'EK3_ENABLE': (1, 'AP_Int8', 'EKF3有効'),
    'EK3_SRC1_POSXY': (3, 'AP_Int8', 'GPS (水平位置)'),
    'EK3_SRC1_VELXY': (3, 'AP_Int8', 'GPS (水平速度)'),
    'EK3_SRC1_POSZ': (3, 'AP_Int8', 'GPS (垂直位置)'),
    'EK3_SRC1_VELZ': (3, 'AP_Int8', 'GPS (垂直速度)'),
    'EK3_SRC1_YAW': (3, 'AP_Int8', 'GPS+コンパス'),

    # ── EKF3: ゲート（厳しめで信頼度向上） ──
    'EK3_GPS_CHECK': (1, 'AP_Int8', 'GPS健全性チェック'),
    'EK3_POS_I_GATE': (5, 'AP_Int16', '位置イノベーションゲート'),
    'EK3_VEL_I_GATE': (5, 'AP_Int16', '速度イノベーションゲート'),
    'EK3_HGT_I_GATE': (5, 'AP_Int16', '高度イノベーションゲート'),
    'EK3_GLITCH_RAD': (5, 'AP_Int8', 'GPS Glitch検出半径 [m]'),
    'EK3_PRIMARY': (-1, 'AP_Int8', '自動切替(粘着なし)'),

    # ── EKF3: ノイズ（GPS信頼時は小さめ） ──
    'EK3_POSNE_M_NSE': (0.3, 'AP_Float', '水平位置ノイズ [m]'),
    'EK3_VELNE_M_NSE': (0.5, 'AP_Float', '水平速度ノイズ [m/s]'),
    'EK3_VELD_M_NSE': (0.7, 'AP_Float', '垂直速度ノイズ [m/s]'),
    'EK3_ALT_M_NSE': (3.0, 'AP_Float', '高度ノイズ [m]'),

    # ── シリアル: TELEM1 のみ MAVLink、他は既存維持 ──
    'SERIAL1_PROTOCOL': (2, 'AP_Int8', 'TELEM1: MAVLink2'),
    'SERIAL1_BAUD': (921600, 'AP_Int32', 'TELEM1: 921600'),
    'BRD_SER1_RTSCTS': (2, 'AP_Int8', 'TELEM1: ハードウェアフロー制御'),

    # ── フェイルセーフ ──
    'FS_EKF_ACTION': (1, 'AP_Int8', 'EKF失敗時: Land'),
    'FS_EKF_THRESH': (0.8, 'AP_Float', 'EKF信頼度閾値'),

    # ── アーミングチェック ──
    'ARMING_CHECK': (1, 'AP_Int32', '全チェック有効'),
    'ARMING_RUDDER': (0, 'AP_Int8', 'ラダーアーミング無効'),
}

# === ユーティリティ ===

MAV_PARAM_TYPE_MAP = {
    'AP_Int8':  mavutil.mavlink.MAV_PARAM_TYPE_INT8,
    'AP_Int16': mavutil.mavlink.MAV_PARAM_TYPE_INT16,
    'AP_Int32': mavutil.mavlink.MAV_PARAM_TYPE_INT32,
    'AP_Float': mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
}


def clear_message_buffer(timeout=0.1):
    n = 0
    while master.recv_match(blocking=False, timeout=timeout):
        n += 1
    if n:
        print(f"  [バッファ] {n}個クリア")
    return n


def wait_for_param_ack(param_name, expected_value, timeout=2.0):
    start = time.time()
    while time.time() - start < timeout:
        msg = master.recv_match(type='PARAM_VALUE', blocking=False, timeout=0.05)
        if msg is None:
            continue
        got_name = msg.to_dict().get('param_id', '').rstrip('\x00')
        if got_name == param_name:
            return msg.to_dict().get('param_value', None), True
    return None, False


def set_parameter_reliable(param_name, param_value, param_type, max_retries=5):
    mav_type = MAV_PARAM_TYPE_MAP.get(param_type, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)

    for attempt in range(max_retries):
        clear_message_buffer(timeout=0.05)

        master.mav.param_set_send(
            master.target_system, master.target_component,
            param_name.encode('utf-8'), float(param_value), mav_type)

        time.sleep(0.2)
        received_value, received = wait_for_param_ack(param_name, param_value, timeout=1.5)

        if received and received_value is not None:
            if abs(float(received_value) - float(param_value)) < 0.0001:
                return True, received_value
            if attempt < max_retries - 1:
                print(f'    ⚠️ 値不一致 リトライ{attempt+1}/{max_retries}: 設定={param_value}, 確認={received_value}')
                time.sleep(0.3)
            else:
                print(f'    ❌ 最終失敗: 設定={param_value}, 確認={received_value}')
                return False, received_value
        else:
            if attempt < max_retries - 1:
                print(f'    ⚠️ タイムアウト リトライ{attempt+1}/{max_retries}')
                time.sleep(0.3)
            else:
                print(f'    ❌ タイムアウト（設定失敗）')
                return False, None
    return False, None


def verify_parameter(param_name, expected_value):
    clear_message_buffer(timeout=0.05)
    master.mav.param_request_read_send(
        master.target_system, master.target_component,
        param_name.encode('utf-8'), -1)
    time.sleep(0.15)
    received_value, received = wait_for_param_ack(param_name, expected_value, timeout=1.5)
    if received and received_value is not None:
        return float(received_value), True
    return None, False


# === メイン ===

print(f"\n{'='*60}")
print(f"GPS/RTK パラメータ設定 ({len(params_to_set)}項目)")
print(f"{'='*60}")

failed_params = {}
total = len(params_to_set)
success_count = 0

for idx, (param_name, (param_value, param_type, param_comment)) in enumerate(params_to_set.items(), 1):
    print(f"\n[{idx}/{total}] {param_name}")
    print(f"  設定値: {param_value}  ({param_comment})")

    success, received_value = set_parameter_reliable(param_name, param_value, param_type)

    if success:
        print(f"  ✅ 確認値: {received_value}")
        success_count += 1
    else:
        print(f"  ❌ 設定失敗")
        failed_params[param_name] = (param_value, param_type, param_comment)

print(f"\n{'='*60}")
print(f"1回目: {success_count}/{total} 成功")
print(f"{'='*60}")

# FC側の処理完了待ち
time.sleep(2)

# EEPROM保存
print("\nEEPROMに保存中...")
master.mav.command_long_send(
    master.target_system, master.target_component,
    mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE, 0, 1, 0, 0, 0, 0, 0, 0)

ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=10)
eeprom_saved = False
if ack and ack.command == mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE:
    if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print('✅ EEPROM保存成功')
        eeprom_saved = True
    else:
        print(f'⚠️ EEPROM保存結果: {ack.result}')
else:
    print('⚠️ ACK未受信')

time.sleep(2)

# 失敗したパラメータの再設定
if failed_params:
    print(f"\n{'='*60}")
    print(f"⚠️ {len(failed_params)}個のパラメータを再設定...")
    print(f"{'='*60}")

    for param_name, (param_value, param_type, param_comment) in failed_params.items():
        print(f"\n再設定: {param_name}")
        success, received_value = set_parameter_reliable(param_name, param_value, param_type, max_retries=3)
        if success:
            print(f"  ✅ 再設定成功: {received_value}")
        else:
            print(f"  ❌ 再設定失敗")

    print("\n再度EEPROM保存...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE, 0, 1, 0, 0, 0, 0, 0, 0)
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    if ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print('✅ 再保存成功')
    else:
        print('⚠️ 再保存失敗')

# 最終確認
print(f"\n{'='*60}")
print("最終確認...")
print(f"{'='*60}")

final_ok = 0
final_fail = []

for param_name, (param_value, param_type, param_comment) in params_to_set.items():
    received_value, verified = verify_parameter(param_name, param_value)
    if verified and received_value is not None:
        if abs(float(received_value) - float(param_value)) < 0.0001:
            print(f"✅ {param_name} = {received_value}")
            final_ok += 1
        else:
            print(f"❌ {param_name} = {received_value} (期待: {param_value})")
            final_fail.append(param_name)
    else:
        print(f"❌ {param_name} (取得失敗)")
        final_fail.append(param_name)

print(f"\n{'='*60}")
print(f"最終結果: {final_ok}/{total} 成功")
if final_fail:
    print(f"失敗: {len(final_fail)}個")
    for n in final_fail[:10]:
        print(f"  - {n}")

if not final_fail and eeprom_saved:
    print("✅ 全パラメータ設定・保存完了")
print(f"{'='*60}")
print("\n完了")
