import time
from pymavlink import mavutil

def tune_altitude_response_latest():
    """最新ArduPilot対応の高度制御応答性向上設定"""
    
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
    master.wait_heartbeat()
    print(f"接続確認: システム {master.target_system}")
    
    # 最新バージョン対応の高度制御パラメータ（確認済みのもののみ）
    altitude_params = {
        # 位置制御ゲイン（応答性向上）
        'PSC_POSZ_P': 2.0,        # 高度位置制御Pゲイン
        
        # 速度制御ゲイン（応答性向上）
        'PSC_VELZ_P': 8.0,        # 高度速度制御Pゲイン
        'PSC_VELZ_I': 15.0,       # 高度速度制御Iゲイン
        'PSC_VELZ_D': 0.02,       # 高度速度制御Dゲイン
        
        # 加速度制御ゲイン（応答性向上）
        'PSC_ACCZ_P': 1.0,        # 高度加速度制御Pゲイン
        'PSC_ACCZ_I': 2.0,        # 高度加速度制御Iゲイン
        
        # 代替パラメータ（存在すれば設定）
        'PSC_VELZ_FLT': 10.0,     # PSC_VELZ_FLTVの代替
        'PSC_ACCZ_FLT': 10.0,     # PSC_ACCZ_FLTAの代替
    }
    
    # 削除されたパラメータの代替設定
    deprecated_alternatives = {
        # PSC_VELZ_MAX, PSC_ACCZ_MAXは他のパラメータで代替
        'WPNAV_SPEED_UP': 300,    # 上昇速度制限（cm/s）
        'WPNAV_SPEED_DN': 150,    # 下降速度制限（cm/s）  
        'WPNAV_ACCEL_Z': 300,     # 垂直加速度制限（cm/s/s）
    }
    
    # 全パラメータを統合
    all_params = {**altitude_params, **deprecated_alternatives}
    
    def safe_param_name(param_id):
        try:
            if isinstance(param_id, bytes):
                return param_id.decode('utf-8').rstrip('\x00')
            else:
                return str(param_id).rstrip('\x00')
        except:
            return str(param_id)
    
    print("=" * 70)
    print("最新ArduPilot対応：高度制御応答性向上設定")
    print("=" * 70)
    
    success_count = 0
    for param_id, param_value in all_params.items():
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
            
            # 応答確認（タイムアウト短縮で効率化）
            found = False
            for attempt in range(3):  # 最大3回試行
                message = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
                if message:
                    param_name = safe_param_name(message.param_id)
                    if param_name.upper() == param_id.upper():
                        print(f"確認: {param_name} = {message.param_value}")
                        success_count += 1
                        found = True
                        break
                    # 別のパラメータの応答の場合は続行
                    
            if not found:
                print(f"タイムアウト: {param_id} (削除された可能性)")
                
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
    print(f"設定完了！ ({success_count}/{len(all_params)} 成功)")
    print("=" * 70)
    
    # 変更された内容の説明
    print("最新バージョン対応の変更内容:")
    print("  成功したパラメータ:")
    print("    PSC_POSZ_P: 位置制御強化")
    print("    PSC_VELZ_P/I/D: 速度制御強化")
    print("    PSC_ACCZ_P/I: 加速度制御強化")
    
    print("  削除されたパラメータの代替:")
    print("    PSC_VELZ_MAX → WPNAV_SPEED_UP (上昇速度)")
    print("    PSC_ACCZ_MAX → WPNAV_ACCEL_Z (垂直加速度)")
    print("    PSC_VELZ_FLTV → PSC_VELZ_FLT (フィルター)")
    
    print("\n期待される効果:")
    print("  - 高度指令に対する応答速度向上")
    print("  - 外乱に対するクイックな復帰")
    print("  - より安定した位置追従")

def check_available_altitude_params():
    """利用可能な高度制御パラメータを確認"""
    
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
    master.wait_heartbeat()
    
    # 確認したいパラメータリスト
    params_to_check = [
        'PSC_POSZ_P', 'PSC_VELZ_P', 'PSC_VELZ_I', 'PSC_VELZ_D',
        'PSC_ACCZ_P', 'PSC_ACCZ_I',
        'PSC_VELZ_FLT', 'PSC_VELZ_FLTV', 'PSC_ACCZ_FLT', 'PSC_ACCZ_FLTA',
        'PSC_VELZ_MAX', 'PSC_ACCZ_MAX',
        'WPNAV_SPEED_UP', 'WPNAV_SPEED_DN', 'WPNAV_ACCEL_Z'
    ]
    
    print("利用可能な高度制御パラメータ確認:")
    print("-" * 50)
    
    for param in params_to_check:
        try:
            master.mav.param_request_read_send(
                master.target_system,
                master.target_component,
                param.encode('utf-8'),
                -1
            )
            
            msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
            if msg:
                try:
                    if isinstance(msg.param_id, bytes):
                        name = msg.param_id.decode('utf-8').rstrip('\x00')
                    else:
                        name = str(msg.param_id).rstrip('\x00')
                except:
                    name = str(msg.param_id)
                
                if name.upper() == param.upper():
                    print(f"  ✅ {name}: {msg.param_value}")
                else:
                    print(f"  ❌ {param}: 存在しない")
            else:
                print(f"  ❌ {param}: タイムアウト")
                
        except Exception as e:
            print(f"  ❌ {param}: エラー - {e}")
        
        time.sleep(0.1)

if __name__ == "__main__":
    print("利用可能パラメータの確認:")
    check_available_altitude_params()
    
    print("\n" + "="*70)
    input("確認完了。高度制御設定を実行しますか？ (Enter)")
    
    tune_altitude_response_latest()
