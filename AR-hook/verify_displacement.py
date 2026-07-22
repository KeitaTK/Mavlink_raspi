#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
verify_displacement.py - 2つの実際の実行結果CSVファイルを比較し、
Motive真値とARカメラ推定値のそれぞれから荷物の変位（移動量）を導出して精度を検証するスクリプト。
"""
import os
import sys
sys.stdout.reconfigure(encoding='utf-8')
import math
import csv
from pathlib import Path

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

def load_and_process_csv(filepath):
    """CSVを読み込み、Motiveドローン座標を用いてAR荷物絶対位置を補正した座標リストを返す"""
    print(f"📖 ファイル読み込み中: {os.path.basename(filepath)}")
    
    motive_drone = []
    motive_cargo = []
    ar_cargo_uncorr = []
    ar_cargo_corr = []
    
    with open(filepath, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for idx, row in enumerate(reader):
            try:
                # 荷物が検出されている行のみを処理
                if int(row['Cargo_Detected']) != 1:
                    continue
                
                # 各座標値の取得
                mdx = float(row['Motive_Drone_X'])
                mdy = float(row['Motive_Drone_Y'])
                mdz = float(row['Motive_Drone_Z'])
                
                mcx = float(row['Motive_Cargo_X'])
                mcy = float(row['Motive_Cargo_Y'])
                mcz = float(row['Motive_Cargo_Z'])
                
                ax = float(row['Cargo_X'])
                ay = float(row['Cargo_Y'])
                az = float(row['Cargo_Z'])
                
                gx = float(row['GPS_X'])
                gy = float(row['GPS_Y'])
                gz = float(row['GPS_Z'])
                
                # 無効な座標値（すべて0など）はスキップ
                if (mdx == 0.0 and mdy == 0.0 and mdz == 0.0) or (mcx == 0.0 and mcy == 0.0 and mcz == 0.0):
                    continue
                
                # 補正後のAR荷物世界座標の算出
                # Cargo_corrected = Motive_Drone + (Cargo_AR - GPS_drone)
                acx_corr = mdx + (ax - gx)
                acy_corr = mdy + (ay - gy)
                acz_corr = mdz + (az - gz)
                
                motive_drone.append([mdx, mdy, mdz])
                motive_cargo.append([mcx, mcy, mcz])
                ar_cargo_uncorr.append([ax, ay, az])
                ar_cargo_corr.append([acx_corr, acy_corr, acz_corr])
                
            except (ValueError, KeyError) as e:
                continue
                
    if not motive_cargo:
        print(f"⚠ {os.path.basename(filepath)} に有効な荷物データが見つかりませんでした。")
        return None
        
    print(f"   -> 有効サンプル数: {len(motive_cargo)}")
    
    # 平均値の算出
    res = {
        'drone_motive': np.mean(motive_drone, axis=0) if HAS_NUMPY else [sum(x)/len(x) for x in zip(*motive_drone)],
        'cargo_motive': np.mean(motive_cargo, axis=0) if HAS_NUMPY else [sum(x)/len(x) for x in zip(*motive_cargo)],
        'cargo_ar_uncorr': np.mean(ar_cargo_uncorr, axis=0) if HAS_NUMPY else [sum(x)/len(x) for x in zip(*ar_cargo_uncorr)],
        'cargo_ar_corr': np.mean(ar_cargo_corr, axis=0) if HAS_NUMPY else [sum(x)/len(x) for x in zip(*ar_cargo_corr)],
        'raw_data': {
            'drone_motive': motive_drone,
            'cargo_motive': motive_cargo,
            'cargo_ar_uncorr': ar_cargo_uncorr,
            'cargo_ar_corr': ar_cargo_corr
        }
    }
    return res

def main():
    # デフォルトのファイルパス（2回分の実行ログ）
    default_run1 = "C:/Users/Ushida/Downloads/20260722_061003_1.csv"
    default_run2 = "C:/Users/Ushida/Downloads/20260722_061106_1.csv"
    
    run1_path = sys.argv[1] if len(sys.argv) > 1 else default_run1
    run2_path = sys.argv[2] if len(sys.argv) > 2 else default_run2
    
    if not os.path.exists(run1_path) or not os.path.exists(run2_path):
        print("❌ 指定されたCSVファイルが見つかりません。パスを確認してください。")
        print(f"Run 1: {run1_path}")
        print(f"Run 2: {run2_path}")
        return
        
    print("=" * 60)
    print(" 2回分のCSVログの比較による荷物の変位導出解析")
    print("=" * 60)
    
    r1 = load_and_process_csv(run1_path)
    r2 = load_and_process_csv(run2_path)
    
    if not r1 or not r2:
        print("❌ いずれかのファイルの解析に失敗したため、変位の導出を中止します。")
        return
        
    # 変位（Displacement）の計算
    # 1. Motive真値の変位
    d_motive = np.array(r2['cargo_motive']) - np.array(r1['cargo_motive'])
    d_motive_len = np.linalg.norm(d_motive)
    
    # 2. 補正後ARの変位
    d_ar_corr = np.array(r2['cargo_ar_corr']) - np.array(r1['cargo_ar_corr'])
    d_ar_corr_len = np.linalg.norm(d_ar_corr)
    
    # 3. 補正前ARの変位
    d_ar_uncorr = np.array(r2['cargo_ar_uncorr']) - np.array(r1['cargo_ar_uncorr'])
    d_ar_uncorr_len = np.linalg.norm(d_ar_uncorr)
    
    # 変位誤差
    err_corr = d_ar_corr - d_motive
    err_corr_len = np.linalg.norm(err_corr)
    
    err_uncorr = d_ar_uncorr - d_motive
    err_uncorr_len = np.linalg.norm(err_uncorr)
    
    print("\n" + "="*60)
    print(" ■ 荷物の平均絶対座標の導出結果 [m]")
    print("="*60)
    print("【Run 1】")
    print(f"  Motive 真値座標   : [X:{r1['cargo_motive'][0]:.4f}, Y:{r1['cargo_motive'][1]:.4f}, Z:{r1['cargo_motive'][2]:.4f}]")
    print(f"  AR 補正後座標     : [X:{r1['cargo_ar_corr'][0]:.4f}, Y:{r1['cargo_ar_corr'][1]:.4f}, Z:{r1['cargo_ar_corr'][2]:.4f}]")
    print(f"  AR 補正前（生）   : [X:{r1['cargo_ar_uncorr'][0]:.4f}, Y:{r1['cargo_ar_uncorr'][1]:.4f}, Z:{r1['cargo_ar_uncorr'][2]:.4f}]")
    print("\n【Run 2】")
    print(f"  Motive 真値座標   : [X:{r2['cargo_motive'][0]:.4f}, Y:{r2['cargo_motive'][1]:.4f}, Z:{r2['cargo_motive'][2]:.4f}]")
    print(f"  AR 補正後座標     : [X:{r2['cargo_ar_corr'][0]:.4f}, Y:{r2['cargo_ar_corr'][1]:.4f}, Z:{r2['cargo_ar_corr'][2]:.4f}]")
    print(f"  AR 補正前（生）   : [X:{r2['cargo_ar_uncorr'][0]:.4f}, Y:{r2['cargo_ar_uncorr'][1]:.4f}, Z:{r2['cargo_ar_uncorr'][2]:.4f}]")
    
    print("\n" + "="*60)
    print(" ■ 荷物の変位（Run 1 -> Run 2）の比較結果 [m]")
    print("="*60)
    print(f"  Motive真値変位 d_motive : [dX:{d_motive[0]:.4f}, dY:{d_motive[1]:.4f}, dZ:{d_motive[2]:.4f}] | 距離: {d_motive_len*100:.2f} cm")
    print(f"  AR補正後変位   d_ar_corr: [dX:{d_ar_corr[0]:.4f}, dY:{d_ar_corr[1]:.4f}, dZ:{d_ar_corr[2]:.4f}] | 距離: {d_ar_corr_len*100:.2f} cm")
    print(f"  AR補正前変位   d_uncorr : [dX:{d_ar_uncorr[0]:.4f}, dY:{d_ar_uncorr[1]:.4f}, dZ:{d_ar_uncorr[2]:.4f}] | 距離: {d_ar_uncorr_len*100:.2f} cm")
    print("-" * 60)
    print(f"  補正後変位誤差 (AR_corr - Motive) : [eX:{err_corr[0]:.4f}, eY:{err_corr[1]:.4f}, eZ:{err_corr[2]:.4f}] | 距離誤差: {err_corr_len*100:.2f} cm")
    print(f"  補正前変位誤差 (AR_raw - Motive)  : [eX:{err_uncorr[0]:.4f}, eY:{err_uncorr[1]:.4f}, eZ:{err_uncorr[2]:.4f}] | 距離誤差: {err_uncorr_len*100:.2f} cm")
    print("="*60)
    
    # グラフ描画
    if HAS_MATPLOTLIB and HAS_NUMPY:
        plot_path = Path(run2_path).parent / "displacement_comparison.png"
        print(f"\n📊 グラフ画像を保存中: {plot_path}")
        
        plt.figure(figsize=(10, 8))
        
        # サンプル点プロット
        # Run 1
        m_c1 = np.array(r1['raw_data']['cargo_motive'])
        a_c1 = np.array(r1['raw_data']['cargo_ar_corr'])
        plt.scatter(m_c1[:, 0], m_c1[:, 1], color='red', alpha=0.1, label='Run 1: Motive Samples')
        plt.scatter(a_c1[:, 0], a_c1[:, 1], color='orange', alpha=0.1, label='Run 1: AR Corrected Samples')
        
        # Run 2
        m_c2 = np.array(r2['raw_data']['cargo_motive'])
        a_c2 = np.array(r2['raw_data']['cargo_ar_corr'])
        plt.scatter(m_c2[:, 0], m_c2[:, 1], color='blue', alpha=0.1, label='Run 2: Motive Samples')
        plt.scatter(a_c2[:, 0], a_c2[:, 1], color='cyan', alpha=0.1, label='Run 2: AR Corrected Samples')
        
        # 平均位置プロット
        plt.plot(r1['cargo_motive'][0], r1['cargo_motive'][1], 'ro', markersize=10, markeredgecolor='black', label='Run 1: Motive Mean')
        plt.plot(r1['cargo_ar_corr'][0], r1['cargo_ar_corr'][1], 's', color='darkorange', markersize=10, markeredgecolor='black', label='Run 1: AR Corrected Mean')
        
        plt.plot(r2['cargo_motive'][0], r2['cargo_motive'][1], 'bo', markersize=10, markeredgecolor='black', label='Run 2: Motive Mean')
        plt.plot(r2['cargo_ar_corr'][0], r2['cargo_ar_corr'][1], 's', color='darkblue', markersize=10, markeredgecolor='black', label='Run 2: AR Corrected Mean')
        
        # 変位ベクトルの描画（矢印）
        plt.quiver(r1['cargo_motive'][0], r1['cargo_motive'][1], d_motive[0], d_motive[1], 
                   angles='xy', scale_units='xy', scale=1, color='red', width=0.008, 
                   label=f'Motive Displacement Vector ({d_motive_len*100:.2f} cm)')
                   
        plt.quiver(r1['cargo_ar_corr'][0], r1['cargo_ar_corr'][1], d_ar_corr[0], d_ar_corr[1], 
                   angles='xy', scale_units='xy', scale=1, color='blue', width=0.008, 
                   label=f'AR Corrected Vector ({d_ar_corr_len*100:.2f} cm)')
        
        plt.xlabel('X (East) Position [m]', fontsize=11)
        plt.ylabel('Y (North) Position [m]', fontsize=11)
        plt.title('Cargo Horizontal Position and Displacement Comparison\n(Motive Ground Truth vs Corrected AR Camera)', fontsize=13, fontweight='bold')
        plt.grid(True, linestyle='--', alpha=0.5)
        plt.legend(loc='lower left', fontsize=9)
        plt.axis('equal')
        
        plt.tight_layout()
        plt.savefig(plot_path, dpi=150)
        print(f"✓ グラフ保存完了: {plot_path}")
    else:
        print("ℹ️ Matplotlib/NumPy がないため、グラフ描画をスキップしました。")

if __name__ == '__main__':
    main()
