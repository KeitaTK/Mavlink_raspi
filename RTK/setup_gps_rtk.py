#!/usr/bin/env python3
"""
ArduPilot GPS/RTK パラメータ設定スクリプト (Pixhawk6C)

GPS_INJECT_TO を 0 にして TELEM1 への RTCM 注入を停止し、
GPSの信頼度を高める設定を行う。

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
params = {
    # ── RTCM注入: 全ポート注入を停止 ──
    'GPS_INJECT_TO': (0, 'AP_Int8', 'RTCM注入先ポート (0=無効, 127=全ポート)'),

    # ── GPS 自動設定 ──
    'GPS_AUTO_CONFIG': (2, 'AP_Int8', '自動設定 (2=DroneCANのみ)'),
    'GPS_AUTO_SWITCH': (1, 'AP_Int8', 'GPS自動切替有効'),

    # ── GPS 測位信頼度 ──
    'GPS_GNSS_MODE': (67, 'AP_Int8',
        'GNSSモード (67=GPS+GLONASS+SBAS+QZSS, 日本向け)'),
    'GPS_MIN_DGPS': (100, 'AP_Int16',
        'DGPS最小衛星数 (sats×100, 100=1sats, デフォルト100)'),
    'GPS_HDOP_GOOD': (120, 'AP_Int16',
        '良好HDOP閾値 (×100, 120=1.2, デフォルト140)'),
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
    'EK3_ALT_M_NSE': (3.0, 'AP_Float', '高度ノイズ [m] (GPS信頼時は小さめ)'),

    # ── シリアルポート: TELEM1のみMAVLink、他は維持 ──
    'SERIAL1_PROTOCOL': (2, 'AP_Int8', 'TELEM1: MAVLink2'),
    'SERIAL1_BAUD': (921600, 'AP_Int32', 'TELEM1: 921600 (ﾊｰﾄﾞｳｪｱﾌﾛｰ制御)'),
    # SERIAL3/4 は既存値 (5=GPS) を維持
    # SERIAL2 は既存値 (2=MAVLink2, 双葉) を維持

    # ── フェイルセーフ: EKF失敗時の動作 ──
    'FS_EKF_ACTION': (1, 'AP_Int8', 'EKF失敗時: Land'),
    'FS_EKF_THRESH': (0.8, 'AP_Float', 'EKF信頼度閾値'),
    'FS_OPTIONS': (0, 'AP_Float', '追加FS無効'),

    # ── アーミングチェック: GPS必須 ──
    'ARMING_CHECK': (1, 'AP_Int32', '全チェック有効'),
    'ARMING_RUDDER': (0, 'AP_Int8', 'ラダーアーミング無効'),
}

# === パラメータ書き込みロジック ===
MAV_PARAM_TYPE_MAP = {
    'AP_Int8': mavutil.mavlink.MAV_PARAM_TYPE_INT8,
    'AP_Int16': mavutil.mavlink.MAV_PARAM_TYPE_INT16,
    'AP_Int32': mavutil.mavlink.MAV_PARAM_TYPE_INT32,
    'AP_Float': mavutil.mavlink.MAV_PARAM_TYPE_REAL32,
}


def clear_buffer():
    n = 0
    while master.recv_match(blocking=False, timeout=0.05):
        n += 1
    return n


def set_param(name, value, ptype, retries=5):
    mav_type = MAV_PARAM_TYPE_MAP[ptype]
    for attempt in range(retries):
        clear_buffer()
        master.mav.param_set_send(
            master.target_system, master.target_component,
            name.encode(), float(value), mav_type)
        time.sleep(0.2)

        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1.5)
        if msg:
            got = msg.param_value
            if abs(float(got) - float(value)) < 0.001:
                return True, got
        if attempt < retries - 1:
            time.sleep(0.3)
    return False, None


def verify_param(name):
    clear_buffer()
    master.mav.param_request_read_send(
        master.target_system, master.target_component, name.encode(), -1)
    time.sleep(0.15)
    msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1.5)
    if msg:
        return float(msg.param_value), True
    return None, False


# === 実行 ===
print(f"\n{'='*60}")
print(f"GPS/RTK パラメータ設定 ({len(params)}項目)")
print(f"{'='*60}")

ok = 0
fail = []

for i, (name, (val, ptype, desc)) in enumerate(params.items(), 1):
    print(f"\n[{i}/{len(params)}] {name} = {val}  ({desc})")
    success, got = set_param(name, val, ptype)
    if success:
        print(f"  ✅ {got}")
        ok += 1
    else:
        print(f"  ❌ 失敗 (got={got})")
        fail.append(name)

print(f"\n{'='*60}")
print(f"結果: {ok}/{len(params)} 成功")
if fail:
    print(f"失敗: {fail}")
print(f"{'='*60}")

# EEPROM保存
if input("\nEEPROMに保存しますか (y/n): ").strip().lower() == 'y':
    print("保存中...")
    master.mav.command_long_send(
        master.target_system, master.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE, 0, 1, 0, 0, 0, 0, 0, 0)
    ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    if ack and ack.result == 0:
        print("✅ EEPROM保存完了")
    else:
        print("⚠️ 保存失敗（アーム中だと拒否されます）")

print("\n完了")
