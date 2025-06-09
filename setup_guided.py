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
{
    # --- RCチャンネル設定 ---
    # RC11のスイッチをGUIDEDモードの有効/無効化に割り当てる
    'RC11_OPTION': 18,        # 18: Guidedモードを有効化[1]

    # --- GUIDEDモード専用の設定 ---
    # Pythonスクリプトからの指令が途絶えた場合の安全設定
    'GUID_TIMEOUT': 3,        # 3秒間、新しい指令がない場合、機体は停止する (秒単位)[1]
    'GUID_OPTIONS': 0,        # GUIDEDモード中のオプション (0=デフォルト、パイロットのスティック操作によるオーバーライドを許可)

    # --- ナビゲーション設定 (GUIDEDモードでも使用される) ---
    # 目標地点への移動速度や到達判定に関する設定
    'WPNAV_RADIUS': 10,       # 目標地点への到達判定半径 (cm)
    'WPNAV_SPEED': 50,        # 水平方向の最大移動速度 (cm/s)

    # --- スロットル設定 (共通) ---
    'THR_DZ': 200,            # スロットルのデッドバンド (PWM値で±100)

    # --- 位置・速度制御ゲイン (Loiter, Auto, Guidedで共有される重要なパラメータ) ---
    # 機体の応答性や安定性に直接影響する
    'PSC_POSXY_P': 2.0,       # 水平位置制御のPゲイン (値を上げると目標位置に留まろうとする力が強くなる)
    'PSC_VELXY_P': 1.5,       # 水平速度制御のPゲイン
    'PSC_VELXY_I': 3.0,       # 水平速度制御のIゲイン (値を上げると風など外乱への抵抗が強くなる)
    'PSC_POSZ_P': 4.0,        # 高度位置制御のPゲイン
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
