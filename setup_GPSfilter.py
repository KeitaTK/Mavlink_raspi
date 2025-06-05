import time
from pymavlink import mavutil

def setup_gps_altitude_only():
    """GPS高度のみ使用 + フィルタ軽量化設定"""
    
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
    master.wait_heartbeat()
    print(f"接続確認: システム {master.target_system}")
    
    # GPS高度専用 + フィルタ軽量化パラメータ
    gps_altitude_params = {
        # EKF高度ソース設定（GPS専用）
        'EK3_SRC1_POSZ': 0,       # GPS高度のみ使用
        'EK3_SRC1_VELZ': 0,       # GPS垂直速度使用
        
        # GPS高度の信頼度向上
        'EK3_ALT_M_NSE': 0.5,     # GPS高度測定ノイズ（小さく = 信頼度高）
        'EK3_HGT_I_GATE': 500,    # 高度イノベーションゲート（大きく）
        
        # 気圧計無効化（GPS専用のため）
        'EK3_OGN_HGT_MASK': 0,    # 気圧計補正無効
        
        # パイロットスロットルフィルタ軽量化
        'PILOT_THR_FILT': 0,      # スロットルフィルタ無効（応答性最優先）
        
        # 高度フィルタ軽量化（制御ゲインは変更せず）
        'INS_GYRO_FILTER': 5,     # ジャイロフィルタ軽量化（Hz）
        'INS_ACCEL_FILTER': 5,    # 加速度フィルタ軽量化（Hz）
        
        # EKF更新レート向上
        'EK3_IMU_MASK': 3,        # 複数IMU使用で精度向上
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
    print("GPS高度専用 + フィルタ軽量化設定")
    print("=" * 60)
    
    success_count = 0
    for param_id, param_value in gps_altitude_params.items():
        print(f"{param_id}を{param_value}に設定中...")
        
        # パラメータタイプを自動判定
        if isinstance(param_value, float):
            param_type = mavutil.mavlink.MAV_PARAM_TYPE_REAL32
        else:
            param_type = mavutil.mavlink.MAV_PARAM_TYPE_INT32
        
        master.mav.param_set_send(
            master.target_system,
            master.target_component,
            param_id.encode('utf-8'),
            param_value,
            param_type
        )
        
        message = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=3)
        if message:
            param_name = safe_param_name(message.param_id)
            if param_name.upper() == param_id.upper():
                print(f"確認: {param_name} = {message.param_value}")
                success_count += 1
            else:
                retry_msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=2)
                if retry_msg and safe_param_name(retry_msg.param_id).upper() == param_id.upper():
                    print(f"確認: {safe_param_name(retry_msg.param_id)} = {retry_msg.param_value}")
                    success_count += 1
                else:
                    print(f"タイムアウト: {param_id}")
        else:
            print(f"タイムアウト: {param_id}")
        
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
    
    ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=5)
    if ack_msg and ack_msg.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
        print("パラメータ保存完了")
    else:
        print("保存結果不明")
    
    print("=" * 60)
    print(f"設定完了！ ({success_count}/{len(gps_altitude_params)} 成功)")
    print("=" * 60)
    
    # 設定内容の説明
    print("設定された内容:")
    print("【GPS高度専用設定】")
    print("  EK3_SRC1_POSZ = 0: GPS高度のみ使用（気圧計無効）")
    print("  EK3_ALT_M_NSE = 0.5: GPS高度の信頼度向上")
    print("  EK3_HGT_I_GATE = 500: 高度変化許容範囲拡大")
    
    print("\n【フィルタ軽量化】")
    print("  PILOT_THR_FILT = 0: スロットルフィルタ無効")
    print("  INS_GYRO_FILTER = 5Hz: ジャイロフィルタ軽量化")
    print("  INS_ACCEL_FILTER = 5Hz: 加速度フィルタ軽量化")
    
    print("\n【効果】")
    print("  - GPS高度への直接的な応答性向上")
    print("  - 気圧計による遅延除去")
    print("  - フィルタ軽量化による応答性向上")
    print("  - 制御ゲインは変更なし（安定性維持）")

if __name__ == "__main__":
    setup_gps_altitude_only()
