import time
from pymavlink import mavutil

# MAVLinkコネクションを作成
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)

# コマンドを送信する前にハートビートを待つ
print("ハートビートを待機中...")
master.wait_heartbeat()
print(f"ハートビート受信: システム {master.target_system} コンポーネント {master.target_component}")

# Motiveモーションキャプチャ→GPS室内飛行用パラメータ
parameters = {
    # GPS設定（MAVLink GPS使用）
    'GPS1_TYPE': (14, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
    'GPS_AUTO_SWITCH': (0, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
    
    # EKF3設定（モーションキャプチャ用）
    'EK3_SRC1_YAW': (6, mavutil.mavlink.MAV_PARAM_TYPE_INT32),        # External Navigation
    'EK3_SRC1_POSZ': (6, mavutil.mavlink.MAV_PARAM_TYPE_INT32),       # External Navigation
    'EK3_GPS_CHECK': (1, mavutil.mavlink.MAV_PARAM_TYPE_INT32),       # 最小限チェック
    'EK3_POSNE_M_NSE': (0.01, mavutil.mavlink.MAV_PARAM_TYPE_REAL32), # 1cm精度
    'EK3_YAW_M_NSE': (0.1, mavutil.mavlink.MAV_PARAM_TYPE_REAL32),    # ヨー角ノイズ
    
    # コンパス無効化（ヨー角はモーションキャプチャのみ）
    'COMPASS_ENABLE': (0, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
    
    # フェイルセーフ設定（GPS切れ時はマニュアル制御）
    'FS_EKF_ACTION': (4, mavutil.mavlink.MAV_PARAM_TYPE_INT32),
    
    # RTL高度設定（室内用に低く設定）
    'RTL_ALT': (100, mavutil.mavlink.MAV_PARAM_TYPE_INT32),           # 1m（100cm）
}

print("=" * 60)
print("Motiveモーションキャプチャ→GPS室内飛行用パラメータ設定")
print("=" * 60)

# SET_GPS_GLOBAL_ORIGIN送信（EKF原点設定）
print("GPS_GLOBAL_ORIGINを設定中...")
# 基準GPS座標（NatNetClientの設定と同じ）
ref_lat = int(36.0757800 * 1e7)  # 緯度（degE7）
ref_lon = int(136.2132900 * 1e7) # 経度（degE7）
ref_alt = int(0 * 1000)          # 高度（mm）

master.mav.set_gps_global_origin_send(
    master.target_system,
    ref_lat,
    ref_lon,
    ref_alt,
    0  # time_usec (0 = unknown)
)

# GPS_GLOBAL_ORIGIN設定確認
msg = master.recv_match(type='GPS_GLOBAL_ORIGIN', blocking=True, timeout=5)
if msg:
    print(f"GPS_GLOBAL_ORIGIN設定完了: lat={msg.latitude/1e7:.7f}, lon={msg.longitude/1e7:.7f}")
else:
    print("WARNING: GPS_GLOBAL_ORIGIN設定の確認タイムアウト")

print("-" * 60)

# すべてのパラメータを設定
for param_id, (param_value, param_type) in parameters.items():
    print(f"{param_id}を{param_value}に設定中...")
    
    # パラメータを設定
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        param_id.encode('utf-8'),
        param_value,
        param_type
    )
    
    # ACK（応答）を読み取る
    message = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=5)
    if message:
        message_dict = message.to_dict()
        param_name = message_dict['param_id'].decode('utf-8').rstrip('\x00')
        print(f"✅ 確認: {param_name} = {message_dict['param_value']}")
    else:
        print(f"⚠️  警告: {param_id}の設定確認タイムアウト")
    
    # 次のコマンドの前に少し待機
    time.sleep(0.5)

print("-" * 60)

# パラメータをEEPROMに永続保存
print("パラメータをEEPROMに保存中...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE,
    0,
    1, 0, 0, 0, 0, 0, 0
)

# 保存コマンドの応答確認
ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=10)
if ack_msg and ack_msg.command == mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE:
    if ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("✅ パラメータ保存完了")
    else:
        print(f"⚠️  パラメータ保存エラー: {ack_msg.result}")
else:
    print("⚠️  パラメータ保存確認タイムアウト")

print("=" * 60)
print("設定完了！以下の手順を実行してください：")
print("1. ArduPilotを再起動")
print("2. Motiveデータブリッジを開始")
print("3. GuidedモードまたはLoiterモードでテスト飛行")
print("=" * 60)

# 追加確認：設定されたパラメータの読み戻し
print("\n設定確認（読み戻し）:")
for param_id in parameters.keys():
    master.mav.param_request_read_send(
        master.target_system,
        master.target_component,
        param_id.encode('utf-8'),
        -1
    )
    
    msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
    if msg:
        param_name = msg.param_id.decode('utf-8').rstrip('\x00')
        print(f"  {param_name}: {msg.param_value}")
    time.sleep(0.2)

print("すべての設定が完了しました！")
