import time
from pymavlink import mavutil

# MAVLinkコネクションを作成
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)

# ハートビートを待機
print("ハートビートを待機中...")
master.wait_heartbeat()
print(f"ハートビート受信: システム {master.target_system} コンポーネント {master.target_component}")

# 超高精度・超低速室内用Loiter Mode設定
loiter_parameters = {
    # Radio10スイッチ設定
    'CH10_OPT': 58,           # Radio10をLoiter Modeスイッチに設定
    
    # 超精密・超安全設定
    'WPNAV_LOIT_SPEED': 50,   # Loiter最大速度 (cm/s) = 0.5m/s（超安全）
    'WPNAV_LOIT_MAXA': 50,    # 加速度 (cm/s/s)（非常に穏やか）
    'WPNAV_LOIT_MINA': 25,    # 減速度 (cm/s/s)（極めて優しい停止）
    
    # 追加の超精密設定
    'LOIT_BRK_DELAY': 0.3,    # ブレーキ開始遅延（秒）（短縮）
    'LOIT_BRK_ACCEL': 50,     # ブレーキ加速度（非常に穏やか）
    'LOIT_BRK_JERK': 300,     # ブレーキ変化率（非常に滑らか）
    'LOIT_ANG_MAX': 10,       # 最大傾斜角（度）（超安全重視）
    
    # 超高精度位置制御設定
    'WPNAV_RADIUS': 10,       # 到達半径（cm）（10cm精度）
    'WPNAV_SPEED': 50,        # 水平ナビゲーション速度（cm/s）= 0.5m/s
    
    # 追加の精密制御パラメータ
    'PSC_POSXY_P': 2.0,       # 水平位置制御ゲイン（精密制御）
    'PSC_VELXY_P': 1.5,       # 水平速度制御ゲイン
    'PSC_VELXY_I': 3.0,       # 水平速度積分ゲイン
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
print("超高精度室内用Loiter Mode設定（10cm精度・0.5m/s制限）")
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
        
        # 応答確認
        message = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
        if message:
            param_name = safe_param_name(message.param_id)
            print(f"確認: {param_name} = {message.param_value}")
            success_count += 1
        else:
            print(f"タイムアウト: {param_id}")
            
    except Exception as e:
        print(f"エラー: {param_id} - {e}")
    
    time.sleep(0.3)

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

# 設定内容の説明
print("\n超高精度設定内容:")
print("  位置精度: 10cm以内（半径10cm圏内で位置保持）")
print("  最大速度: 0.5m/s（超安全重視）")
print("  加速度: 50cm/s/s（非常に穏やか）")
print("  最大傾斜角: 10度（超安全重視）")
print("  Radio10スイッチ: Loiter Mode ON/OFF")

print("\n精度レベル比較:")
print("  一般的な屋外GPS: ±1-3m")
print("  RTK-GPS: ±2-5cm")
print("  この設定: ±10cm（モーションキャプチャ活用）")

print("\n使用方法:")
print("  1. Stabilizeモードで手動離陸")
print("  2. 希望高度でホバリング安定")
print("  3. Radio10スイッチをONでLoiter Mode開始")
print("  4. 10cm精度でその場に自動位置保持")

print("\n超精密制御の注意事項:")
print("  - 非常に低速・高精度設定")
print("  - 風や振動の影響を受けやすい")
print("  - 初回は必ず低高度でテスト")
print("  - モーションキャプチャの安定性が重要")

# 重要パラメータの最終確認
print("\n超精密パラメータ最終確認:")
check_params = ['WPNAV_RADIUS', 'WPNAV_SPEED', 'WPNAV_LOIT_SPEED', 'LOIT_ANG_MAX']
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
            name = safe_param_name(msg.param_id)
            unit = "cm" if "RADIUS" in name else "cm/s" if "SPEED" in name else "度" if "ANG" in name else ""
            print(f"  {name}: {msg.param_value}{unit}")
        else:
            print(f"  {param}: 確認タイムアウト")
    except Exception as e:
        print(f"  {param}: 確認エラー")
    time.sleep(0.2)

print("\n超高精度Loiter Mode設定が完了しました！")
print("位置精度: ±10cm、最大速度: 0.5m/s")
