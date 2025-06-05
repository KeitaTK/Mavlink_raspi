import time
from pymavlink import mavutil

def tune_altitude_response():
    """高度制御の応答性を向上させる設定"""
    
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
    master.wait_heartbeat()
    print(f"接続確認: システム {master.target_system}")
    
    # 高度制御応答性向上パラメータ
    altitude_params = {
        # 位置制御ゲイン（クイック応答）
        'PSC_POSZ_P': 2.0,        # 高度位置制御Pゲイン（上げる）
        
        # 速度制御ゲイン（強化）
        'PSC_VELZ_P': 8.0,        # 高度速度制御Pゲイン
        'PSC_VELZ_I': 15.0,       # 高度速度制御Iゲイン
        'PSC_VELZ_D': 0.02,       # 高度速度制御Dゲイン（追加）
        
        # 加速度制御ゲイン（調整）
        'PSC_ACCZ_P': 1.0,        # 高度加速度制御Pゲイン
        'PSC_ACCZ_I': 2.0,        # 高度加速度制御Iゲイン
        
        # フィルタ設定（応答性重視）
        'PSC_VELZ_FLTV': 10.0,    # 速度フィルタ（高周波カット）
        'PSC_ACCZ_FLTA': 10.0,    # 加速度フィルタ
        
        # 制限値の調整
        'PSC_VELZ_MAX': 300,      # 最大上昇速度（cm/s）（上げる）
        'PSC_ACCZ_MAX': 300,      # 最大加速度（cm/s/s）（上げる）
    }
    
    def safe_param_name(param_id):
        try:
            if isinstance(param_id, bytes):
                return param_id.decode('utf-8').rstrip('\x00')
            else:
                return str(param_id).rstrip('\x00')
        except:
            return str(param_id)
    
    print("=" * 60)
    print("高度制御応答性向上設定")
    print("=" * 60)
    
    success_count = 0
    for param_id, param_value in altitude_params.items():
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
            message = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
            if message:
                param_name = safe_param_name(message.param_id)
                if param_name.upper() == param_id.upper():
                    print(f"確認: {param_name} = {message.param_value}")
                    success_count += 1
                else:
                    # 再試行
                    retry_msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
                    if retry_msg and safe_param_name(retry_msg.param_id).upper() == param_id.upper():
                        print(f"再試行成功: {safe_param_name(retry_msg.param_id)} = {retry_msg.param_value}")
                        success_count += 1
            else:
                print(f"タイムアウト: {param_id}")
                
        except Exception as e:
            print(f"エラー: {param_id} - {e}")
        
        time.sleep(0.3)
    
    # EEPROM保存
    print("-" * 60)
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
    
    print("=" * 60)
    print(f"設定完了！ ({success_count}/{len(altitude_params)} 成功)")
    print("=" * 60)
    
    # 設定効果の説明
    print("変更内容:")
    print("  PSC_POSZ_P: 1.0 → 2.0 (位置制御強化)")
    print("  PSC_VELZ_P: 5.0 → 8.0 (速度制御強化)")
    print("  PSC_VELZ_I: 10.0 → 15.0 (定常偏差除去強化)")
    print("  PSC_VELZ_MAX: → 300 (最大上昇速度向上)")
    
    print("\n期待される効果:")
    print("  - 高度指令に対する応答速度向上")
    print("  - 外乱に対するクイックな復帰")
    print("  - 位置追従精度の向上")
    
    print("\n注意事項:")
    print("  - 振動が発生する場合はPゲインを下げる")
    print("  - オーバーシュートする場合はDゲインを調整")
    print("  - 段階的にテスト飛行を実施")

if __name__ == "__main__":
    tune_altitude_response()
