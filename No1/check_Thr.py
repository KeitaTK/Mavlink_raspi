import time
from pymavlink import mavutil

def check_mot_thst_hover():
    """MOT_THST_HOVERパラメータを確認する"""
    
    # MAVLinkコネクションを作成
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
    
    # ハートビートを待つ
    print("ハートビートを待機中...")
    master.wait_heartbeat()
    print(f"ハートビート受信: システム {master.target_system} コンポーネント {master.target_component}")
    
    # MOT_THST_HOVERパラメータを要求
    print("MOT_THST_HOVERパラメータを要求中...")
    master.mav.param_request_read_send(
        master.target_system,
        master.target_component,
        b'MOT_THST_HOVER',
        -1
    )
    
    # 応答を待つ
    start_time = time.time()
    timeout = 5
    
    while time.time() - start_time < timeout:
        try:
            message = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
            if message:
                # パラメータ名を安全に取得
                try:
                    if isinstance(message.param_id, bytes):
                        param_name = message.param_id.decode('utf-8').rstrip('\x00')
                    else:
                        param_name = str(message.param_id).rstrip('\x00')
                except:
                    param_name = str(message.param_id)
                
                # MOT_THST_HOVERパラメータが見つかった場合
                if param_name == 'MOT_THST_HOVER':
                    hover_value = message.param_value
                    hover_percent = hover_value * 100
                    
                    print("=" * 50)
                    print("MOT_THST_HOVER パラメータ情報")
                    print("=" * 50)
                    print(f"パラメータ名: {param_name}")
                    print(f"値: {hover_value:.4f}")
                    print(f"パーセント: {hover_percent:.1f}%")
                    print("=" * 50)
                    
                    # 値の評価
                    if hover_value < 0.2:
                        print("評価: 値が低すぎます（< 20%）")
                        print("推奨: より重いペイロードまたはモーター出力確認が必要")
                    elif hover_value > 0.8:
                        print("評価: 値が高すぎます（> 80%）")
                        print("推奨: より軽量化またはモーター/プロペラのアップグレードが必要")
                    else:
                        print("評価: 正常な範囲内（20% - 80%）")
                    
                    # スロットル関連の推奨事項
                    print("\nスロットル設定の推奨:")
                    print(f"- 送信機のホバリング位置: 約{hover_percent:.0f}%")
                    print("- Loiterモード: この値付近で高度を維持")
                    print("- 学習機能: 自動飛行モードで自動調整される")
                    
                    return hover_value
                    
                # 他のパラメータが来た場合はスキップ
                else:
                    print(f"他のパラメータを受信: {param_name} = {message.param_value}")
                    continue
                    
        except Exception as e:
            print(f"受信エラー: {e}")
            continue
    
    print("タイムアウト: MOT_THST_HOVERパラメータの取得に失敗しました")
    return None

def check_related_parameters():
    """関連パラメータも確認"""
    
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)
    master.wait_heartbeat()
    
    # 関連パラメータリスト
    related_params = [
        'MOT_THST_HOVER',
        'MOT_SPIN_ARM',
        'MOT_SPIN_MIN',
        'MOT_BAT_VOLT_MAX',
        'MOT_BAT_VOLT_MIN'
    ]
    
    print("\n関連パラメータ一覧:")
    print("-" * 60)
    
    for param_name in related_params:
        master.mav.param_request_read_send(
            master.target_system,
            master.target_component,
            param_name.encode('utf-8'),
            -1
        )
        
        # 応答待ち
        found = False
        for _ in range(10):  # 最大10回試行
            try:
                message = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
                if message:
                    try:
                        if isinstance(message.param_id, bytes):
                            received_name = message.param_id.decode('utf-8').rstrip('\x00')
                        else:
                            received_name = str(message.param_id).rstrip('\x00')
                    except:
                        received_name = str(message.param_id)
                    
                    if received_name == param_name:
                        if param_name == 'MOT_THST_HOVER':
                            print(f"{param_name}: {message.param_value:.4f} ({message.param_value*100:.1f}%)")
                        else:
                            print(f"{param_name}: {message.param_value:.4f}")
                        found = True
                        break
            except:
                continue
        
        if not found:
            print(f"{param_name}: 取得失敗")
        
        time.sleep(0.2)

if __name__ == "__main__":
    print("MOT_THST_HOVER確認プログラム")
    print("=" * 50)
    
    # メインのMOT_THST_HOVER確認
    hover_value = check_mot_thst_hover()
    
    if hover_value is not None:
        # 関連パラメータも確認
        check_related_parameters()
        
        print("\n使用方法:")
        print("1. この値が適切かどうか確認")
        print("2. 必要に応じて手動調整")
        print("3. 自動飛行モードで学習させる")
    else:
        print("パラメータ取得に失敗しました")
        print("ArduPilotとの接続を確認してください")
