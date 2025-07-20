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

def parse_pwm_data(text):
    """
    PWM M0: 1488 -> 0.4880 の形式からPWM値のみを抽出
    """
    try:
        if "PWM M" not in text or "->" not in text:
            return None
        
        # モーター番号を抽出
        motor_part = text.split("PWM M")[1].split(":")[0]
        motor_id = f"M{motor_part}"
        
        # PWM値のみ抽出
        pwm_part = text.split("->")[0].split(":")[-1].strip()
        pwm_value = int(pwm_part)
        
        return {
            'motor': motor_id,
            'pwm': pwm_value
        }
    except Exception as e:
        return None

try:
    print("PWM値記録プログラム開始")
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
    
    print("PWMデータ収集中... (Ctrl+Cで終了)")
    print("=" * 70)
    
    # PWMデータ収集用変数
    pwm_samples = {'M0': [], 'M1': [], 'M2': [], 'M3': []}
    total_samples = 0
    
    while running:
        msg = master.recv_match(blocking=True, timeout=1)
        
        if msg is not None:
            current_time = datetime.now()
            msg_type = msg.get_type()
            
            if msg_type == 'STATUSTEXT':
                text = msg.text.strip()
                
                # PWMデータのみ処理
                if "PWM M" in text and "->" in text:
                    pwm_info = parse_pwm_data(text)
                    if pwm_info:
                        motor = pwm_info['motor']
                        pwm_value = pwm_info['pwm']
                        
                        # データ収集
                        if motor in pwm_samples:
                            pwm_samples[motor].append(pwm_value)
                            total_samples += 1
                            
                            print(f"{current_time.strftime('%H:%M:%S.%f')[:-3]} : [PWM] {motor}:{pwm_value}μs (#{len(pwm_samples[motor])})")

except Exception as e:
    print(f"エラー: {e}")

finally:
    try:
        if 'master' in locals():
            master.close()
    except:
        pass
    
    # PWM統計結果表示
    print("\n" + "="*70)
    print("ホバリング時PWM値統計結果")
    print("="*70)
    
    if total_samples > 0:
        print(f"総PWMサンプル数: {total_samples}")
        
        motor_counts = {motor: len(samples) for motor, samples in pwm_samples.items()}
        print(f"各モーターサンプル数: {motor_counts}")
        print(f"測定時間: 約 {max(motor_counts.values()) / 40:.1f} 秒 (40Hzで推定)")
        print()
        
        # 各モーターのPWM統計
        print("各モーターのPWM統計:")
        print("-" * 50)
        
        motor_stats = {}
        for motor in ['M0', 'M1', 'M2', 'M3']:
            if motor in pwm_samples and pwm_samples[motor]:
                samples = pwm_samples[motor]
                avg = sum(samples) / len(samples)
                min_val = min(samples)
                max_val = max(samples)
                
                motor_stats[motor] = {
                    'avg': avg,
                    'min': min_val,
                    'max': max_val,
                    'range': max_val - min_val
                }
                
                print(f"{motor}: 平均={avg:.0f}μs, 最小={min_val}μs, 最大={max_val}μs, 範囲={max_val-min_val}μs")
            else:
                print(f"{motor}: データなし")
        
        # 全体統計
        if motor_stats:
            print()
            print("全体統計:")
            print("-" * 50)
            
            all_avgs = [stats['avg'] for stats in motor_stats.values()]
            all_ranges = [stats['range'] for stats in motor_stats.values()]
            
            overall_avg = sum(all_avgs) / len(all_avgs)
            avg_range = sum(all_ranges) / len(all_ranges)
            
            print(f"全モーター平均PWM: {overall_avg:.0f}μs")
            print(f"平均PWM変動範囲: {avg_range:.0f}μs")
            
            # PWMバランス分析
            pwm_std = (sum([(avg - overall_avg)**2 for avg in all_avgs]) / len(all_avgs)) ** 0.5
            print(f"モーター間PWM偏差: {pwm_std:.0f}μs")
            
            if pwm_std < 10:
                print("→ PWMバランス: 良好")
            elif pwm_std < 25:
                print("→ PWMバランス: 要調整")
            else:
                print("→ PWMバランス: 要修正")
            
            print()
            print("ホバリングPWM基準値:")
            print("-" * 50)
            print(f"基準PWM値 (平均): {overall_avg:.0f}μs")
            print(f"PWM許容範囲: ±{avg_range/2:.0f}μs")
            
    else:
        print("PWMデータが取得できませんでした。")
        print("AP_Observerが正常に動作しているか確認してください。")
    
    print("\nPWM記録終了")
