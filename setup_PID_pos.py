import time
from pymavlink import mavutil

def tune_altitude_response_verified():
    """確認済みパラメータのみで高度制御応答性向上"""
    
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
    master.wait_heartbeat()
    print(f"接続確認: システム {master.target_system}")
    
    # 存在確認済みの高度制御パラメータのみ
    verified_altitude_params = {
        # 位置制御ゲイン（応答性向上）
        'PSC_POSZ_P': 2.0,        # 高度位置制御Pゲイン
        
        # 速度制御ゲイン（応答性向上）
        'PSC_VELZ_P': 8.0,        # 高度速度制御Pゲイン
        'PSC_VELZ_I': 15.0,       # 高度速度制御Iゲイン
        'PSC_VELZ_D': 0.02,       # 高度速度制御Dゲイン
        
        # 加速度制御ゲイン（応答性向上）
        'PSC_ACCZ_P': 1.0,        # 高度加速度制御Pゲイン
        'PSC_ACCZ_I': 2.0,        # 高度加速度制御Iゲイン
        
        # ナビゲーション制限（代替制御）
        'WPNAV_SPEED_UP': 300,    # 上昇速度制限（cm/s）
        'WPNAV_SPEED_DN': 150,    # 下降速度制限（cm/s）
        'WPNAV_ACCEL_Z': 300,     # 垂直加速度制限（cm/s/s）
    }
    
    def safe_param_name(param_id):
        try:
            if isinstance(param_id, bytes):
                return param_id.decode('utf-8').rstrip('\x00')
            else:
                return str(param_id).rstrip('\x00')
        except:
            return str(param_id)
    
    print("=" * 70)
    print("確認済みパラメータのみ：高度制御応答性向上設定")
    print("=" * 70)
    
    success_count = 0
    for param_id, param_value in verified_altitude_params.items():
        print(f"{param_id}を{param_value}に設定中...")
        
        try:
            # パラメータ設定
            param_type = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            master.mav.param_set_send(
                master.target_system,
                master.target_component,
                param_id.encode('utf-8'),
                param_value,
                param_type
            )
            
            # 応答確認
            found = False
            for attempt in range(3):
                message = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
                if message:
                    param_name = safe_param_name(message.param_id)
                    if param_name.upper() == param_id.upper():
                        print(f"確認: {param_name} = {message.param_value}")
                        success_count += 1
                        found = True
                        break
                    
            if not found:
                print(f"タイムアウト: {param_id}")
                
        except Exception as e:
            print(f"エラー: {param_id} - {e}")
        
        time.sleep(0.2)
    
    # EEPROM保存
    print("-" * 70)
    print("パラメータをEEPROMに保存中...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE,
        0, 1, 0, 0, 0, 0, 0, 0
    )
    
    try:
        ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
        if ack_msg and ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
            print("パラメータ保存完了")
        else:
            print("保存結果不明")
    except:
        print("保存確認タイムアウト")
    
    print("=" * 70)
    print(f"設定完了！ ({success_count}/{len(verified_altitude_params)} 成功)")
    print("=" * 70)
    
    # 削除されたパラメータについての説明
    print("削除されたパラメータについて:")
    print("  ❌ PSC_VELZ_FLT/FLTV: 最新版では別の方法でフィルター制御")
    print("  ❌ PSC_ACCZ_FLT/FLTA: 最新版では別の方法でフィルター制御")
    print("  ❌ PSC_VELZ_MAX: WPNAV_SPEED_UPで代替")
    print("  ❌ PSC_ACCZ_MAX: WPNAV_ACCEL_Zで代替")
    
    print("\n設定された有効パラメータ:")
    print("  ✅ PSC_POSZ_P: 位置制御強化 (2.0)")
    print("  ✅ PSC_VELZ_P/I/D: 速度制御強化 (8.0/15.0/0.02)")
    print("  ✅ PSC_ACCZ_P/I: 加速度制御強化 (1.0/2.0)")
    print("  ✅ WPNAV制限値: 速度・加速度制限 (300/150/300)")
    
    print("\n期待される効果:")
    print("  - 高度指令に対する応答速度向上")
    print("  - より積極的な位置制御")
    print("  - 外乱からの素早い復帰")

def minimal_altitude_tuning():
    """最小限の高度制御調整（確実に動作）"""
    
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
    master.wait_heartbeat()
    
    # 最小限のコアパラメータのみ
    core_params = {
        'PSC_POSZ_P': 2.0,        # 位置制御強化
        'PSC_VELZ_P': 8.0,        # 速度制御強化
        'PSC_VELZ_I': 15.0,       # 積分制御強化
    }
    
    print("最小限の高度制御調整:")
    print("-" * 40)
    
    for param_id, param_value in core_params.items():
        try:
            master.mav.param_set_send(
                master.target_system,
                master.target_component,
                param_id.encode('utf-8'),
                param_value,
                mavutil.mavlink.MAV_PARAM_TYPE_REAL32
            )
            
            msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
            if msg:
                try:
                    if isinstance(msg.param_id, bytes):
                        name = msg.param_id.decode('utf-8').rstrip('\x00')
                    else:
                        name = str(msg.param_id).rstrip('\x00')
                except:
                    name = str(msg.param_id)
                
                if name.upper() == param_id.upper():
                    print(f"✅ {name}: {msg.param_value}")
                else:
                    print(f"❌ {param_id}: パラメータ名不一致")
            else:
                print(f"❌ {param_id}: タイムアウト")
                
        except Exception as e:
            print(f"❌ {param_id}: エラー - {e}")
        
        time.sleep(0.3)

if __name__ == "__main__":
    print("1. 確認済みパラメータで設定")
    print("2. 最小限のコアパラメータのみ設定")
    
    choice = input("選択 (1 or 2): ").strip()
    
    if choice == "1":
        tune_altitude_response_verified()
    elif choice == "2":
        minimal_altitude_tuning()
    else:
        print("無効な選択")
