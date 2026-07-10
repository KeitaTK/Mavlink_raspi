#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_accuracy.py - Motiveの真値とAR-hook1_test.pyのカメラ推定値・Pixhawkテレメトリを比較し、
追跡精度（MAE、RMSE、標準偏差）を検証・可視化するスクリプト。
"""
import os
import sys
import glob
import math
import csv
from pathlib import Path

# NumPy, Pandas, Matplotlibのインポート（利用できない場合はフォールバック）
try:
    import numpy as np
    HAS_NUMPY = True
except ImportError:
    HAS_NUMPY = False

try:
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

def wrap_angle_rad(angle):
    """角度を [-pi, pi] の範囲に正規化"""
    return (angle + math.pi) % (2 * math.pi) - math.pi

def get_latest_csv(directory):
    """指定されたディレクトリから最新のCSVファイルを検索"""
    path = Path(directory)
    csv_files = list(path.glob("*_1.csv"))
    if not csv_files:
        return None
    return max(csv_files, key=os.path.getmtime)

def calculate_metrics(errors):
    """誤差配列からMAE, RMSE, 標準偏差を計算"""
    if not errors:
        return float('nan'), float('nan'), float('nan')
    
    n = len(errors)
    mae = sum(abs(e) for e in errors) / n
    rmse = math.sqrt(sum(e**2 for e in errors) / n)
    
    mean_err = sum(errors) / n
    variance = sum((e - mean_err)**2 for e in errors) / n
    std_dev = math.sqrt(variance)
    
    return mae, rmse, std_dev

def main():
    # ログ保存ディレクトリの設定
    csv_dir = Path.home() / "LOGS_Pixhawk6c"
    
    # 引数があればそれを使用、なければ最新のCSVを検索
    if len(sys.argv) > 1:
        csv_path = Path(sys.argv[1])
    else:
        csv_path = get_latest_csv(csv_dir)
        
    if not csv_path or not csv_path.exists():
        print(f"❌ ログファイルが見つかりません。ディレクトリ: {csv_dir}")
        print("AR-hook1_test.pyを動かしてログCSVを生成してください。")
        return
        
    print(f"🔍 ログファイルを解析中: {csv_path.name}")
    
    # CSVの読み込み
    times = []
    # Pixhawk
    gps_x, gps_y, gps_z = [], [], []
    cargo_x, cargo_y, cargo_z = [], [], []
    cargo_detected = []
    id1_detected = []
    rel_dx, rel_dy, rel_dz = [], [], []
    pix_roll, pix_pitch, pix_yaw = [] ,[], []
    
    # Motive
    mot_drone_x, mot_drone_y, mot_drone_z = [], [], []
    mot_cargo_x, mot_cargo_y, mot_cargo_z = [], [], []
    mot_rel_x, mot_rel_y, mot_rel_z = [], [], []
    mot_roll, mot_pitch, mot_yaw = [], [], []
    
    with open(csv_path, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                times.append(float(row['Time']))
                
                # 検出フラグ
                c_det = int(row['Cargo_Detected'])
                i_det = int(row['ID1_Detected'])
                cargo_detected.append(c_det)
                id1_detected.append(i_det)
                
                # Pixhawk & Camera
                gps_x.append(float(row['GPS_X']))
                gps_y.append(float(row['GPS_Y']))
                gps_z.append(float(row['GPS_Z']))
                
                cargo_x.append(float(row['Cargo_X']) if row['Cargo_X'] else float('nan'))
                cargo_y.append(float(row['Cargo_Y']) if row['Cargo_Y'] else float('nan'))
                cargo_z.append(float(row['Cargo_Z']) if row['Cargo_Z'] else float('nan'))
                
                rel_dx.append(float(row['ID1_to_Cargo_DX']) if row['ID1_to_Cargo_DX'] else float('nan'))
                rel_dy.append(float(row['ID1_to_Cargo_DY']) if row['ID1_to_Cargo_DY'] else float('nan'))
                rel_dz.append(float(row['ID1_to_Cargo_DZ']) if row['ID1_to_Cargo_DZ'] else float('nan'))
                
                pix_roll.append(float(row['Pixhawk_Roll']))
                pix_pitch.append(float(row['Pixhawk_Pitch']))
                pix_yaw.append(float(row['Pixhawk_Yaw']))
                
                # Motive
                mot_drone_x.append(float(row['Motive_Drone_X']))
                mot_drone_y.append(float(row['Motive_Drone_Y']))
                mot_drone_z.append(float(row['Motive_Drone_Z']))
                
                mot_cargo_x.append(float(row['Motive_Cargo_X']))
                mot_cargo_y.append(float(row['Motive_Cargo_Y']))
                mot_cargo_z.append(float(row['Motive_Cargo_Z']))
                
                mot_rel_x.append(float(row['Motive_Rel_X']) if row['Motive_Rel_X'] else float('nan'))
                mot_rel_y.append(float(row['Motive_Rel_Y']) if row['Motive_Rel_Y'] else float('nan'))
                mot_rel_z.append(float(row['Motive_Rel_Z']) if row['Motive_Rel_Z'] else float('nan'))
                
                mot_roll.append(float(row['Motive_Roll']))
                mot_pitch.append(float(row['Motive_Pitch']))
                mot_yaw.append(float(row['Motive_Yaw']))
            except (ValueError, KeyError) as e:
                # 変換エラーや一部欠損行はスキップ
                continue
                
    if not times:
        print("❌ 有効なデータレコードがありません。")
        return
        
    print(f"✓ {len(times)} 件のレコードを読み込みました。")
    
    # 基準時刻からの経過時間(秒)
    t0 = times[0]
    time_sec = [t - t0 for t in times]
    
    # 誤差算出用リストの構築
    err_drone_x, err_drone_y, err_drone_z, err_drone_3d = [], [], [], []
    err_rel_dx, err_rel_dy, err_rel_dz, err_rel_3d = [], [], [], []
    err_roll, err_pitch, err_yaw = [], [], []
    
    for i in range(len(times)):
        # 1. ドローン位置誤差 (GPS vs Motive)
        ex = gps_x[i] - mot_drone_x[i]
        ey = gps_y[i] - mot_drone_y[i]
        ez = gps_z[i] - mot_drone_z[i]
        err_drone_x.append(ex)
        err_drone_y.append(ey)
        err_drone_z.append(ez)
        err_drone_3d.append(math.sqrt(ex**2 + ey**2 + ez**2))
        
        # 2. カメラ相対距離誤差 (ArUco Camera vs Motive)
        # 荷物とID1両方が検出されており、かつMotiveデータが存在するときのみ
        if cargo_detected[i] and id1_detected[i] and not math.isnan(rel_dx[i]) and not math.isnan(mot_rel_x[i]):
            rdx = rel_dx[i] - mot_rel_x[i]
            rdy = rel_dy[i] - mot_rel_y[i]
            rdz = rel_dz[i] - mot_rel_z[i]
            err_rel_dx.append(rdx)
            err_rel_dy.append(rdy)
            err_rel_dz.append(rdz)
            err_rel_3d.append(math.sqrt(rdx**2 + rdy**2 + rdz**2))
            
        # 3. ドローン姿勢誤差 (Pixhawk IMU vs Motive)
        err_roll.append(wrap_angle_rad(pix_roll[i] - mot_roll[i]))
        err_pitch.append(wrap_angle_rad(pix_pitch[i] - mot_pitch[i]))
        err_yaw.append(wrap_angle_rad(pix_yaw[i] - mot_yaw[i]))
        
    # 指標の計算
    drone_x_mae, drone_x_rmse, drone_x_std = calculate_metrics(err_drone_x)
    drone_y_mae, drone_y_rmse, drone_y_std = calculate_metrics(err_drone_y)
    drone_z_mae, drone_z_rmse, drone_z_std = calculate_metrics(err_drone_z)
    drone_3d_mae, drone_3d_rmse, drone_3d_std = calculate_metrics(err_drone_3d)
    
    rel_x_mae, rel_x_rmse, rel_x_std = calculate_metrics(err_rel_dx)
    rel_y_mae, rel_y_rmse, rel_y_std = calculate_metrics(err_rel_dy)
    rel_z_mae, rel_z_rmse, rel_z_std = calculate_metrics(err_rel_dz)
    rel_3d_mae, rel_3d_rmse, rel_3d_std = calculate_metrics(err_rel_3d)
    
    roll_mae, roll_rmse, roll_std = calculate_metrics(err_roll)
    pitch_mae, pitch_rmse, pitch_std = calculate_metrics(err_pitch)
    yaw_mae, yaw_rmse, yaw_std = calculate_metrics(err_yaw)
    
    # コンソール表示 (日本語)
    print("\n" + "="*60)
    print(" 精度検証レポート / Accuracy Verification Report")
    print("="*60)
    print(f"■ ドローン絶対位置誤差 (Pixhawk GPS - Motive Drone) [m]")
    print(f"  X (East)  - MAE: {drone_x_mae:6.3f}, RMSE: {drone_x_rmse:6.3f}, SD: {drone_x_std:6.3f}")
    print(f"  Y (North) - MAE: {drone_y_mae:6.3f}, RMSE: {drone_y_rmse:6.3f}, SD: {drone_y_std:6.3f}")
    print(f"  Z (Up)    - MAE: {drone_z_mae:6.3f}, RMSE: {drone_z_rmse:6.3f}, SD: {drone_z_std:6.3f}")
    print(f"  3D Total  - MAE: {drone_3d_mae:6.3f}, RMSE: {drone_3d_rmse:6.3f}, SD: {drone_3d_std:6.3f}")
    print("-" * 60)
    print(f"■ カメラ相対距離誤差 (ArUco Camera - Motive Cargo Local) [m] (有効サンプル数: {len(err_rel_3d)})")
    print(f"  X (Cargo) - MAE: {rel_x_mae:6.3f}, RMSE: {rel_x_rmse:6.3f}, SD: {rel_x_std:6.3f}")
    print(f"  Y (Cargo) - MAE: {rel_y_mae:6.3f}, RMSE: {rel_y_rmse:6.3f}, SD: {rel_y_std:6.3f}")
    print(f"  Z (Cargo) - MAE: {rel_z_mae:6.3f}, RMSE: {rel_z_rmse:6.3f}, SD: {rel_z_std:6.3f}")
    print(f"  3D Total  - MAE: {rel_3d_mae:6.3f}, RMSE: {rel_3d_rmse:6.3f}, SD: {rel_3d_std:6.3f}")
    print("-" * 60)
    print(f"■ ドローン姿勢角度誤差 (Pixhawk IMU - Motive) [deg]")
    print(f"  Roll      - MAE: {math.degrees(roll_mae):6.2f}, RMSE: {math.degrees(roll_rmse):6.2f}, SD: {math.degrees(roll_std):6.2f}")
    print(f"  Pitch     - MAE: {math.degrees(pitch_mae):6.2f}, RMSE: {math.degrees(pitch_rmse):6.2f}, SD: {math.degrees(pitch_std):6.2f}")
    print(f"  Yaw       - MAE: {math.degrees(yaw_mae):6.2f}, RMSE: {math.degrees(yaw_rmse):6.2f}, SD: {math.degrees(yaw_std):6.2f}")
    print("="*60)
    
    # グラフ生成 (Matplotlibが利用可能かつデータがある場合)
    if HAS_MATPLOTLIB and HAS_NUMPY:
        output_plot_path = csv_path.parent / f"{csv_path.stem}_accuracy_report.png"
        print(f"📊 グラフ画像を生成中: {output_plot_path.name}")
        
        fig, axs = plt.subplots(3, 1, figsize=(10, 12), sharex=True)
        
        # 1. ドローン位置の比較 (X, Y)
        axs[0].plot(time_sec, gps_x, 'r--', label='Pixhawk GPS X (East)', alpha=0.8)
        axs[0].plot(time_sec, mot_drone_x, 'r-', label='Motive Drone X (East)', alpha=0.8)
        axs[0].plot(time_sec, gps_y, 'b--', label='Pixhawk GPS Y (North)', alpha=0.8)
        axs[0].plot(time_sec, mot_drone_y, 'b-', label='Motive Drone Y (North)', alpha=0.8)
        axs[0].set_ylabel('Position [m]')
        axs[0].set_title('Drone Absolute Position (Pixhawk vs Motive)')
        axs[0].grid(True)
        axs[0].legend()
        
        # 2. カメラ相対距離の比較 (DX, DY, DZ)
        axs[1].plot(time_sec, rel_dx, 'r.', label='Camera Rel DX (Cargo X)')
        axs[1].plot(time_sec, mot_rel_x, 'r-', label='Motive Rel DX (Cargo X)', alpha=0.5)
        axs[1].plot(time_sec, rel_dy, 'g.', label='Camera Rel DY (Cargo Y)')
        axs[1].plot(time_sec, mot_rel_y, 'g-', label='Motive Rel DY (Cargo Y)', alpha=0.5)
        axs[1].plot(time_sec, rel_dz, 'b.', label='Camera Rel DZ (Cargo Z)')
        axs[1].plot(time_sec, mot_rel_z, 'b-', label='Motive Rel DZ (Cargo Z)', alpha=0.5)
        axs[1].set_ylabel('Relative Distance [m]')
        axs[1].set_title('Relative Distance (Drone to Cargo Center)')
        axs[1].grid(True)
        axs[1].legend()
        
        # 3. ドローン姿勢の比較 (Yaw)
        pix_yaw_deg = [math.degrees(y) for y in pix_yaw]
        mot_yaw_deg = [math.degrees(y) for y in mot_yaw]
        axs[2].plot(time_sec, pix_yaw_deg, 'g--', label='Pixhawk Yaw')
        axs[2].plot(time_sec, mot_yaw_deg, 'g-', label='Motive Yaw')
        axs[2].set_xlabel('Time elapsed [s]')
        axs[2].set_ylabel('Yaw Angle [deg]')
        axs[2].set_title('Drone Yaw Angle (Pixhawk vs Motive)')
        axs[2].grid(True)
        axs[2].legend()
        
        plt.tight_layout()
        plt.savefig(output_plot_path, dpi=150)
        print(f"✓ グラフ保存完了: {output_plot_path}")
        
    else:
        print("ℹ️ Matplotlib/NumPy がインポートできないため、プロット画像の生成をスキップしました。")
        print("   pip install matplotlib numpy でインストールすると、自動的に画像が出力されます。")

if __name__ == '__main__':
    main()
