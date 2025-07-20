from pymavlink import mavutil
import time
import signal
import sys
from datetime import datetime

running = True

def signal_handler(sig, frame):
    global running
    print('\n終了中...')
    running = False

signal.signal(signal.SIGINT, signal_handler)

def parse_norm_avg_data(text):
    """
    NORM_AVG: 0.3547 (hover: 0.3500) の形式から正規化値を抽出
    """
    try:
        if "NORM_AVG:" not in text:
            return None
        
        # "NORM_AVG: " 以降を取得
        data_part = text.split("NORM_AVG:")[1].strip()
        
        # 現在値を抽出 (hover部分より前)
        if "(hover:" in data_part:
            current_part = data_part.split("(hover:")[0].strip()
            hover_part = data_part.split("(hover:")[1].replace(")", "").strip()
        else:
            current_part = data_part
            hover_part = None
        
        current_value = float(current_part)
        hover_value = float(hover_part) if hover_part else None
        
        return {
            'current': current_value,
            'hover': hover_value
        }
    except Exception as e:
        return None

try:
    print("正規化スロットル値記録プログラム開始")
    print("ホバリング時にプログラムを開始し、ホバリング終了時にCtrl+Cで終了してください")
    print("=" * 70)
    
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=1000000, rtscts=True)
    
    master.wait_heartbeat(timeout=5)
    print(f"Heartbeat received from system {master.target_system}, component {master.target_component}")
    
    # データストリーム要求
    master.mav.request_data_stream_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS,
        10,  # 10Hz
        1    # start
    )
    
    print("正規化値データ収集中... (Ctrl+Cで終了)")
    print("=" * 70)
    
    # データ収集用変数
    norm_samples = []
    hover_values = []
    total_samples = 0
    
    while running:
        msg = master.recv_match(blocking=True, timeout=1)
        
        if msg is not None:
            current_time = datetime.now()
            msg_type = msg.get_type()
            
            if msg_type == 'STATUSTEXT':
                text = msg.text.strip()
                
                # 正規化値データのみ処理
                if "NORM_AVG:" in text:
                    norm_info = parse_norm_avg_data(text)
                    if norm_info:
                        current_norm = norm_info['current']
                        hover_norm = norm_info['hover']
                        
                        # データ収集
                        norm_samples.append(current_norm)
                        if hover_norm is not None:
                            hover_values.append(hover_norm)
                        total_samples += 1
                        
                        print(f"{current_time.strftime('%H:%M:%S.%f')[:-3]} : [NORM] 現在値:{current_norm:.4f} ホバー基準:{hover_norm:.4f if hover_norm else 'N/A'} (#{total_samples})")

except Exception as e:
    print(f"エラー: {e}")

finally:
    try:
        if 'master' in locals():
            master.close()
    except:
        pass
    
    # 正規化値統計結果表示
    print("\n" + "="*70)
    print("ホバリング時正規化スロットル値統計結果")
    print("="*70)
    
    if total_samples > 0:
        print(f"総サンプル数: {total_samples}")
        print(f"測定時間: 約 {total_samples / 40:.1f} 秒 (40Hzで推定)")
        print()
        
        # 正規化値統計
        print("正規化スロットル値統計:")
        print("-" * 50)
        
        if norm_samples:
            avg_norm = sum(norm_samples) / len(norm_samples)
            min_norm = min(norm_samples)
            max_norm = max(norm_samples)
            range_norm = max_norm - min_norm
            
            print(f"平均正規化値: {avg_norm:.4f}")
            print(f"最小正規化値: {min_norm:.4f}")
            print(f"最大正規化値: {max_norm:.4f}")
            print(f"変動範囲: {range_norm:.4f}")
            
            # 変動係数の計算
            std_dev = (sum([(x - avg_norm)**2 for x in norm_samples]) / len(norm_samples)) ** 0.5
            cv = std_dev / avg_norm if avg_norm > 0 else 0
            
            print(f"標準偏差: {std_dev:.4f}")
            print(f"変動係数: {cv:.1%}")
            
        # ホバー基準値統計
        if hover_values:
            print()
            print("ホバー基準値統計:")
            print("-" * 50)
            
            avg_hover = sum(hover_values) / len(hover_values)
            hover_range = max(hover_values) - min(hover_values) if len(set(hover_values)) > 1 else 0
            
            print(f"平均ホバー基準値: {avg_hover:.4f}")
            print(f"ホバー基準値範囲: {hover_range:.4f}")
            
            # 現在値とホバー基準値の比較
            if norm_samples:
                avg_norm = sum(norm_samples) / len(norm_samples)
                deviation_from_hover = avg_norm - avg_hover
                deviation_percent = (deviation_from_hover / avg_hover) * 100 if avg_hover > 0 else 0
                
                print()
                print("ホバー基準値との比較:")
                print("-" * 50)
                print(f"ホバー基準値からの偏差: {deviation_from_hover:+.4f}")
                print(f"偏差率: {deviation_percent:+.1f}%")
                
                if abs(deviation_percent) < 5:
                    print("→ 状態: 良好なホバリング")
                elif abs(deviation_percent) < 15:
                    print("→ 状態: 軽微な負荷変動")
                else:
                    print("→ 状態: 大きな負荷変動")
        
        # 重量推定の参考情報
        if norm_samples and hover_values:
            print()
            print("重量推定参考情報:")
            print("-" * 50)
            avg_norm = sum(norm_samples) / len(norm_samples)
            avg_hover = sum(hover_values) / len(hover_values)
            
            print(f"ホバリング基準スロットル: {avg_hover:.4f}")
            print(f"現在の平均スロットル: {avg_norm:.4f}")
            print(f"スロットル変化率: {((avg_norm - avg_hover) / avg_hover * 100):+.1f}%")
            print()
            print("※重量推定の目安:")
            print("  +10%以上: 荷物積載または上昇中")
            print("  ±5%以内: 通常のホバリング")
            print("  -10%以下: 荷物投下または下降中")
        
    else:
        print("正規化値データが取得できませんでした。")
        print("AP_Observerが正常に動作しているか確認してください。")
    
    print("\n正規化値記録終了")
