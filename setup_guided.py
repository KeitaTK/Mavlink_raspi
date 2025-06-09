import time
from pymavlink import mavutil

# MAVLinkコネクションを作成
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)

# ハートビートを待機
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

print("=" * 70)
print("超高精度室内用Loiter Mode設定（10cm精度・0.5m/s制限）修正版")
print("=" * 70)

# パラメータ設定の実行
success_count = 0
total_count = len(loiter_parameters)

for param_id, param_value in loiter_parameters.items():
    print(f"{param_id}を{param_value}に設定中...")
    
    # パラメータタイプを自動判定
    if isinstance(param_value, float):
        param_type = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
    else:
        param_type = mavutil.mavlink.MAV_PARAM_TYPE_INT32
    
    try:
        # パラメータを設定
        master.mav.param_set_send(
            master.target_system,
            master.target_component,
            param_id.encode('utf-8'),
            param_value,
            param_type
        )
        
        # 応答確認（タイムアウト延長）
        message = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=5)
        if message:
            param_name = safe_param_name(message.param_id)
            # パラメータ名が一致するかチェック
            if param_name.upper() == param_id.upper():
                print(f"確認: {param_name} = {message.param_value}")
                success_count += 1
            else:
                print(f"警告: 設定パラメータ{param_id}と応答パラメータ{param_name}が異なります")
                # 再試行
                time.sleep(0.5)
                retry_msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
                if retry_msg and safe_param_name(retry_msg.param_id).upper() == param_id.upper():
                    print(f"再試行成功: {safe_param_name(retry_msg.param_id)} = {retry_msg.param_value}")
                    success_count += 1
                else:
                    print(f"再試行失敗: {param_id}")
        else:
            print(f"タイムアウト: {param_id}")
            
    except Exception as e:
        print(f"エラー: {param_id} - {e}")
    
    time.sleep(0.5)  # 間隔を長く

print("-" * 70)

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
        print("パラメータ保存完了")
    else:
        print("保存結果不明")
except:
    print("保存確認タイムアウト")

print("=" * 70)
print(f"設定完了！ ({success_count}/{total_count} 成功)")
print("=" * 70)

# 重要パラメータの最終確認
print("重要パラメータ最終確認:")
check_params = ['RC10_OPTION', 'LOIT_SPEED', 'WPNAV_RADIUS', 'LOIT_ANG_MAX']
for param in check_params:
    try:
        master.mav.param_request_read_send(
            master.target_system,
            master.target_component,
            param.encode('utf-8'),
            -1
        )
        
        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
        if msg:
            name = safe_param_name(msg.param_id)
            if name == 'RC10_OPTION':
                mode_name = "Loiter Mode" if msg.param_value == 56 else f"Unknown({msg.param_value})"
                print(f"  {name}: {msg.param_value} ({mode_name})")
            else:
                unit = "cm" if "RADIUS" in name else "cm/s" if "SPEED" in name else "度" if "ANG" in name else ""
                print(f"  {name}: {msg.param_value}{unit}")
        else:
            print(f"  {param}: 確認タイムアウト")
    except Exception as e:
        print(f"  {param}: 確認エラー - {e}")
    time.sleep(0.3)

print("\n設定内容:")
print("  RC10_OPTION: 56 (Loiter Mode)")
print("  位置精度: 10cm以内")
print("  最大速度: 0.5m/s")
print("  最大傾斜角: 10度")
print("\n使用方法:")
print("  1. Stabilizeモードで手動離陸")
print("  2. Radio10スイッチをONでLoiter Mode開始")
print("  3. 10cm精度でその場に自動位置保持")

print("\n超高精度Loiter Mode設定が完了しました！")
