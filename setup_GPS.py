import time
from pymavlink import mavutil

# MAVLinkコネクションを作成
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)

# コマンドを送信する前にハートビートを待つ
print("ハートビートを待機中...")
master.wait_heartbeat()
print(f"ハートビート受信: システム {master.target_system} コンポーネント {master.target_component}")

# Motiveモーションキャプチャ→GPS室内飛行用パラメータ
# 正しいパラメータ設定
parameters = {
    # GPS設定
    'GPS1_TYPE': 14,
    'GPS_AUTO_SWITCH': 0,
    
    # EKF3設定（GPS_INPUT対応、全てMotiveデータ由来）
    'EK3_SRC1_YAW': 2,        # GPS（GPS_INPUTのyawフィールド）
    'EK3_SRC1_POSZ': 0,       # GPS（GPS_INPUTの高精度高度データ）
    'EK3_GPS_CHECK': 1,       # 最小限チェック
    'EK3_POSNE_M_NSE': 0.01,  # 1cm精度
    'EK3_YAW_M_NSE': 0.1,     # ヨー角ノイズ
    
    # コンパス無効化
    'COMPASS_ENABLE': 0,
    
    # フェイルセーフ設定
    'FS_EKF_ACTION': 4,       # EKF失敗時はマニュアル
    
    # RTL高度設定（室内用）
    'RTL_ALT': 100,           # 1m（100cm）
}

def safe_param_name(param_id):
    """パラメータ名を安全に文字列化"""
    try:
        if isinstance(param_id, bytes):
            return param_id.decode('utf-8').rstrip('\x00')
        else:
            return str(param_id).rstrip('\x00')
    except:
        return str(param_id)

print("=" * 60)
print("Motiveモーションキャプチャ→GPS室内飛行用パラメータ設定")
print("=" * 60)

# SET_GPS_GLOBAL_ORIGIN送信（EKF原点設定）
print("GPS_GLOBAL_ORIGINを設定中...")
ref_lat = int(36.0757800 * 1e7)  # 緯度（degE7）
ref_lon = int(136.2132900 * 1e7) # 経度（degE7）
ref_alt = int(0 * 1000)          # 高度（mm）

master.mav.set_gps_global_origin_send(
    master.target_system,
    ref_lat,
    ref_lon,
    ref_alt,
    0
)
time.sleep(1)
print("GPS_GLOBAL_ORIGIN設定完了")

print("-" * 60)

# すべてのパラメータを設定
for param_id, param_value in parameters.items():
    print(f"{param_id}を{param_value}に設定中...")
    
    # パラメータタイプを自動判定
    if isinstance(param_value, float):
        param_type = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    else:
        param_type = mavutil.mavlink.MAV_PARAM_TYPE_INT32
    
    # パラメータを設定
    master.mav.param_set_send(
        master.target_system,
        master.target_component,
        param_id.encode('utf-8'),
        param_value,
        param_type
    )
    
    # 応答確認（エラー対応版）
    try:
        message = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
        if message:
            # パラメータ名を安全に取得
            param_name = safe_param_name(message.param_id)
            print(f"✅ 確認: {param_name} = {message.param_value}")
        else:
            print(f"⚠️  タイムアウト: {param_id}")
    except Exception as e:
        print(f"⚠️  エラー: {param_id} - {e}")
    
    time.sleep(0.3)

print("-" * 60)

# パラメータをEEPROMに保存
print("パラメータをEEPROMに保存中...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE,
    0,
    1, 0, 0, 0, 0, 0, 0
)

# 保存確認
try:
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    if ack_msg and ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("✅ パラメータ保存完了")
    else:
        print("⚠️  保存結果不明")
except:
    print("⚠️  保存確認タイムアウト")

print("=" * 60)
print("設定完了！")
print("1. ArduPilotを再起動してください")
print("2. Motiveデータブリッジを開始してください")  
print("3. GuidedモードまたはLoiterモードでテスト飛行")
print("=" * 60)

# 簡単な確認（エラー対応版）
print("\n最終確認:")
check_params = ['GPS1_TYPE', 'EK3_SRC1_YAW', 'COMPASS_ENABLE']
for param in check_params:
    try:
        master.mav.param_request_read_send(
            master.target_system,
            master.target_component,
            param.encode('utf-8'),
            -1
        )
        
        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
        if msg:
            # パラメータ名を安全に取得
            name = safe_param_name(msg.param_id)
            print(f"  {name}: {msg.param_value}")
        else:
            print(f"  {param}: タイムアウト")
    except Exception as e:
        print(f"  {param}: 確認エラー - {e}")
    time.sleep(0.2)

print("すべての設定が完了しました！")
