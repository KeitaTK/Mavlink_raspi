import time
from pymavlink import mavutil

# MAVLinkコネクションを作成
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)

# コマンドを送信する前にハートビートを待つ
print("ハートビートを待機中...")
master.wait_heartbeat()
print(f"ハートビート受信: システム {master.target_system} コンポーネント {master.target_component}")

# =================================================================
# GUIDEDモード用のパラメータ設定例
# RCチャンネル11をGUIDEDモードの有効化スイッチとして使用する
# =================================================================
parameters = {
    # RC11のスイッチをGUIDEDモードの有効/無効化に割り当てる
    'RC11_OPTION': 55,
    
    # --- GUIDEDモード専用の設定 ---
    'GUID_TIMEOUT': 3,
    'GUID_OPTIONS': 0,
    
    # --- ゆっくり離陸のための垂直速度制御 ---
    'WPNAV_SPEED_UP': 10,       # 超低速上昇: 0.3m/s (デフォルト250→30)
    'WPNAV_ACCEL_Z': 10,        # 超低加速度: 0.25m/s² (デフォルト100→25)
    
    # --- 水平移動も低速に統一 ---
    'WPNAV_SPEED': 30,          # 水平移動も低速: 0.3m/s (50→30)
    'WPNAV_RADIUS': 20,         # より大きな到達判定半径でゆっくり動作
    
    # --- 制御ゲインも穏やかに調整 ---
    'PSC_VELZ_P': 2.0,          # 垂直速度制御を穏やかに (デフォルト5.0→2.0)
    'PSC_ACCZ_P': 0.15,         # 垂直加速度制御を穏やかに (デフォルト0.5→0.15)
    'PSC_ACCZ_I': 0.3,          # 積分ゲインも穏やかに (デフォルト1.0→0.3)
    
    # --- 既存の水平制御設定（そのまま） ---
    'PSC_POSXY_P': 2.0,
    'PSC_VELXY_P': 1.5,
    'PSC_VELXY_I': 3.0,
    'PSC_POSZ_P': 4.0,
    'THR_DZ': 200,
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
