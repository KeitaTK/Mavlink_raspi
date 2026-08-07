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
    """CSVを読み込み、Motive荷物座標、AR荷物座標、およびカメラ座標系での各座標を返す"""
    print(f"📖 ファイル読み込み中: {os.path.basename(filepath)}")
    
    motive_cargo = []
    ar_cargo_uncorr = []
    
    # カメラ座標系データ
    cargo_cam = []
    hand_cam = []
    motive_cargo_cam = []
    motive_hand_cam = []
    
    # オプティカルオフセット校正用サンプル
    calib_samples = []
    
    with open(filepath, 'r', encoding='utf-8') as f:
        reader = csv.DictReader(f)
        for idx, row in enumerate(reader):
            try:
                # 荷物が検出されている行のみを処理
                if int(row['Cargo_Detected']) != 1:
                    continue
                
                # 世界座標系（ENU）データの取得
                mcx = float(row['Motive_Cargo_X'])
                mcy = float(row['Motive_Cargo_Y'])
                mcz = float(row['Motive_Cargo_Z'])
                
                ax = float(row['Cargo_X'])
                ay = float(row['Cargo_Y'])
                az = float(row['Cargo_Z'])
                
                # 無効な座標値（すべて0など）はスキップ
                if mcx == 0.0 and mcy == 0.0 and mcz == 0.0:
                    continue
                
                motive_cargo.append([mcx, mcy, mcz])
                ar_cargo_uncorr.append([ax, ay, az])
                
                # カメラ座標系データの取得
                if 'ID1_Detected' in row and int(row['ID1_Detected']) == 1:
                    hcx = float(row['Hand_Cam_X'])
                    hcy = float(row['Hand_Cam_Y'])
                    hcz = float(row['Hand_Cam_Z'])
                    
                    ccx = float(row['Cargo_Cam_X'])
                    ccy = float(row['Cargo_Cam_Y'])
                    ccz = float(row['Cargo_Cam_Z'])
                    
                    m_ccx = float(row['Motive_Cargo_Cam_X'])
                    m_ccy = float(row['Motive_Cargo_Cam_Y'])
                    m_ccz = float(row['Motive_Cargo_Cam_Z'])
                    
                    m_hcx = float(row['Motive_Hand_Cam_X'])
                    m_hcy = float(row['Motive_Hand_Cam_Y'])
                    m_hcz = float(row['Motive_Hand_Cam_Z'])
                    
                    # すべての値が正常（nanでない）なら追加
                    if not (math.isnan(hcx) or math.isnan(ccx) or math.isnan(m_ccx) or math.isnan(m_hcx)):
                        cargo_cam.append([ccx, ccy, ccz])
                        hand_cam.append([hcx, hcy, hcz])
                        motive_cargo_cam.append([m_ccx, m_ccy, m_ccz])
                        motive_hand_cam.append([m_hcx, m_hcy, m_hcz])
                
                # CAMERA_OPTICAL_OFFSET キャリブレーション用データの収集
                if HAS_NUMPY and 'Motive_Camera_X' in row:
                    mcam_x = float(row['Motive_Camera_X'])
                    mcam_y = float(row['Motive_Camera_Y'])
                    mcam_z = float(row['Motive_Camera_Z'])
                    
                    ccx = float(row['Cargo_Cam_X'])
                    ccy = float(row['Cargo_Cam_Y'])
                    ccz = float(row['Cargo_Cam_Z'])
                    
                    m_ccx = float(row['Motive_Cargo_Cam_X'])
                    m_ccy = float(row['Motive_Cargo_Cam_Y'])
                    m_ccz = float(row['Motive_Cargo_Cam_Z'])
                    
                    if not (math.isnan(mcam_x) or math.isnan(ccx) or math.isnan(m_ccx) or mcam_x == 0.0):
                        v1_cam = np.array([ccx, ccy, ccz], dtype=np.float64)
                        v1_w = np.array([ax - mcam_x, ay - mcam_y, az - mcam_z], dtype=np.float64)
                        
                        v2_cam = np.array([m_ccx, m_ccy, m_ccz], dtype=np.float64)
                        v2_w = np.array([mcx - mcam_x, mcy - mcam_y, mcz - mcam_z], dtype=np.float64)
                        
                        # SVD による回転行列 R の復元 (Camera -> World)
                        A_mat = np.column_stack([v1_cam, v2_cam])
                        B_mat = np.column_stack([v1_w, v2_w])
                        H = A_mat @ B_mat.T
                        U, S, Vt = np.linalg.svd(H)
                        R_mat = Vt.T @ U.T
                        if np.linalg.det(R_mat) < 0:
                            Vt[2, :] *= -1
                            R_mat = Vt.T @ U.T
                            
                        calib_samples.append({
                            'R': R_mat,
                            'mcam': np.array([mcam_x, mcam_y, mcam_z], dtype=np.float64),
                            'mcargo': np.array([mcx, mcy, mcz], dtype=np.float64),
                            'ccam': v1_cam
                        })
                        
            except (ValueError, KeyError) as e:
                continue
                
    if not motive_cargo:
        print(f"⚠ {os.path.basename(filepath)} に有効な荷物データが見つかりませんでした。")
        return None
        
    print(f"   -> 有効世界座標サンプル数: {len(motive_cargo)}")
    print(f"   -> 有効カメラ座標サンプル数: {len(cargo_cam)}")
    
    # 平均値の算出
    res = {
        'cargo_motive': np.mean(motive_cargo, axis=0) if HAS_NUMPY else [sum(x)/len(x) for x in zip(*motive_cargo)],
        'cargo_ar_uncorr': np.mean(ar_cargo_uncorr, axis=0) if HAS_NUMPY else [sum(x)/len(x) for x in zip(*ar_cargo_uncorr)],
        
        'cargo_cam': np.mean(cargo_cam, axis=0) if HAS_NUMPY and len(cargo_cam) > 0 else ([sum(x)/len(x) for x in zip(*cargo_cam)] if len(cargo_cam) > 0 else [float('nan')]*3),
        'hand_cam': np.mean(hand_cam, axis=0) if HAS_NUMPY and len(hand_cam) > 0 else ([sum(x)/len(x) for x in zip(*hand_cam)] if len(hand_cam) > 0 else [float('nan')]*3),
        'motive_cargo_cam': np.mean(motive_cargo_cam, axis=0) if HAS_NUMPY and len(motive_cargo_cam) > 0 else ([sum(x)/len(x) for x in zip(*motive_cargo_cam)] if len(motive_cargo_cam) > 0 else [float('nan')]*3),
        'motive_hand_cam': np.mean(motive_hand_cam, axis=0) if HAS_NUMPY and len(motive_hand_cam) > 0 else ([sum(x)/len(x) for x in zip(*motive_hand_cam)] if len(motive_hand_cam) > 0 else [float('nan')]*3),
        
        'calib_samples': calib_samples,
        
        'raw_data': {
            'cargo_motive': motive_cargo,
            'cargo_ar_uncorr': ar_cargo_uncorr,
            'cargo_cam': cargo_cam,
            'hand_cam': hand_cam,
            'motive_cargo_cam': motive_cargo_cam,
            'motive_hand_cam': motive_hand_cam
        }
    }
    return res

def calibrate_optical_offset(r1, r2, offset_cargo_r1=None, offset_hand_r1=None):
    """
    【方法3: 複数カメラ向きログによる CAMERA_OPTICAL_OFFSET 最適化】
    最小二乗法を用いて、カメラ向きの異なる複数のログデータから
    カメラ光学中心オフセット CAMERA_OPTICAL_OFFSET [X, Y, Z] を自動算出します。
    """
    if not HAS_NUMPY:
        print("⚠ NumPy が利用できないため、CAMERA_OPTICAL_OFFSET のキャリブレーション計算をスキップします。")
        return None
        
    s1 = r1.get('calib_samples', [])
    s2 = r2.get('calib_samples', [])
    all_samples = s1 + s2
    
    if len(all_samples) < 10:
        print("⚠ 有効なキャリブレーションサンプル数が不足しています。")
        return None
        
    A_rows = []
    b_rows = []
    
    for s in all_samples:
        R = s['R']
        y = s['mcargo'] - s['mcam'] - R @ s['ccam']
        A_rows.append(R)
        b_rows.append(y)
        
    A = np.vstack(A_rows)
    b = np.concatenate(b_rows)
    
    # 最小二乗法による大域的最適解の導出 (A * offset = b)
    offset, _, _, _ = np.linalg.lstsq(A, b, rcond=None)
    
    # 校正前後の誤差評価
    errs_before = [np.linalg.norm((s['mcam'] + s['R'] @ s['ccam']) - s['mcargo']) for s in all_samples]
    errs_after = [np.linalg.norm((s['mcam'] + s['R'] @ (s['ccam'] + offset)) - s['mcargo']) for s in all_samples]
    
    err1_before = [np.linalg.norm((s['mcam'] + s['R'] @ s['ccam']) - s['mcargo']) for s in s1] if s1 else []
    err1_after = [np.linalg.norm((s['mcam'] + s['R'] @ (s['ccam'] + offset)) - s['mcargo']) for s in s1] if s1 else []
    
    err2_before = [np.linalg.norm((s['mcam'] + s['R'] @ s['ccam']) - s['mcargo']) for s in s2] if s2 else []
    err2_after = [np.linalg.norm((s['mcam'] + s['R'] @ (s['ccam'] + offset)) - s['mcargo']) for s in s2] if s2 else []
    
    print("\n" + "="*60)
    print(" ■ 【方法3】CAMERA_OPTICAL_OFFSET 最小二乗自動キャリブレーション結果")
    print("   (カメラ向きの異なるログ統合最適化: Run 1 & Run 2)")
    print("="*60)
    print(" 【算出された最適オフセット値】")
    print(f"  CAMERA_OPTICAL_OFFSET (m)  : [X:{offset[0]:.4f}, Y:{offset[1]:.4f}, Z:{offset[2]:.4f}]")
    print(f"  CAMERA_OPTICAL_OFFSET (cm) : [X:{offset[0]*100:.2f} cm, Y:{offset[1]*100:.2f} cm, Z:{offset[2]*100:.2f} cm]")
    print("-" * 60)
    print(" 【3D位置誤差の改善結果】")
    print(f"  全体平均誤差  :  適用前 {np.mean(errs_before)*100:.2f} cm  -->  適用後 {np.mean(errs_after)*100:.2f} cm (改善度: {(np.mean(errs_before)-np.mean(errs_after))*100:+.2f} cm)")
    if s1:
        print(f"  Run 1 平均誤差 :  適用前 {np.mean(err1_before)*100:.2f} cm  -->  適用後 {np.mean(err1_after)*100:.2f} cm")
    if s2:
        print(f"  Run 2 平均誤差 :  適用前 {np.mean(err2_before)*100:.2f} cm  -->  適用後 {np.mean(err2_after)*100:.2f} cm")
    print("-" * 60)
    print(" 💡 【AR-hook1_test.py への貼り付け用コード】")
    print(f" CAMERA_OPTICAL_OFFSET = np.array([{offset[0]:.4f}, {offset[1]:.4f}, {offset[2]:.4f}], dtype=np.float32)")
    print("="*60)

    # 💾 NPZファイルとして自動保存
    try:
        save_path = Path(__file__).parent / "camera_optical_offset.npz"
        opt_arr = offset.astype(np.float32)
        c_off = offset_cargo_r1.astype(np.float32) if 'offset_cargo_r1' in locals() else np.zeros(3, dtype=np.float32)
        h_off = offset_hand_r1.astype(np.float32) if 'offset_hand_r1' in locals() else np.zeros(3, dtype=np.float32)
        
        np.savez(
            save_path,
            optical_offset=opt_arr,
            cargo_offset=c_off,
            hand_offset=h_off
        )
        print(f"💾 キャリブレーションファイルを自動保存しました: {save_path}")
    except Exception as e:
        print(f"⚠ NPZファイルの保存に失敗しました: {e}")

    return offset

def main():
    # デフォルトのファイルパス（2回分の実行ログ）
    default_run1 = "C:/Users/kazzu/Downloads/20260731_131244_1.csv"
    default_run2 = "C:/Users/kazzu/Downloads/20260731_131759_1.csv"
    
    run1_path = sys.argv[1] if len(sys.argv) > 1 else default_run1
    run2_path = sys.argv[2] if len(sys.argv) > 2 else default_run2
    
    if not os.path.exists(run1_path) or not os.path.exists(run2_path):
        print("❌ 指定されたCSVファイルが見つかりません。パスを確認してください。")
        print(f"Run 1: {run1_path}")
        print(f"Run 2: {run2_path}")
        return
        
    print("=" * 60)
    print(" 2回分のCSVログの比較による荷物の変位導出解析 & オプティカルオフセット校正")
    print("=" * 60)
    
    r1 = load_and_process_csv(run1_path)
    r2 = load_and_process_csv(run2_path)
    
    if not r1 or not r2:
        print("❌ いずれかのファイルの解析に失敗したため、処理を中止します。")
        return
        
    # 変位（Displacement）の計算
    # 1. Motive真値の変位
    d_motive = np.array(r2['cargo_motive']) - np.array(r1['cargo_motive'])
    d_motive_len = np.linalg.norm(d_motive)
    
    # 2. 補正前ARの変位
    d_ar_uncorr = np.array(r2['cargo_ar_uncorr']) - np.array(r1['cargo_ar_uncorr'])
    d_ar_uncorr_len = np.linalg.norm(d_ar_uncorr)
    
    # 変位誤差
    err_uncorr = d_ar_uncorr - d_motive
    err_uncorr_len = np.linalg.norm(err_uncorr)
    
    # 精度向上用のカメラ座標系オフセット推奨値 (Offset = Motive_Cam - AR_Cam)
    has_cam_r1 = not np.isnan(r1['cargo_cam'][0])
    has_cam_r2 = not np.isnan(r2['cargo_cam'][0])
    
    if has_cam_r1:
        offset_cargo_r1 = np.array(r1['motive_cargo_cam']) - np.array(r1['cargo_cam'])
        offset_hand_r1 = np.array(r1['motive_hand_cam']) - np.array(r1['hand_cam'])
    else:
        offset_cargo_r1 = np.zeros(3)
        offset_hand_r1 = np.zeros(3)
        
    if has_cam_r2:
        offset_cargo_r2 = np.array(r2['motive_cargo_cam']) - np.array(r2['cargo_cam'])
        offset_hand_r2 = np.array(r2['motive_hand_cam']) - np.array(r2['hand_cam'])
    else:
        offset_cargo_r2 = np.zeros(3)
        offset_hand_r2 = np.zeros(3)
    
    print("\n" + "="*60)
    print(" ■ 荷物の平均絶対世界座標（ENU）の導出結果 [m]")
    print("="*60)
    print("【Run 1】")
    print(f"  Motive 真値座標   : [X:{r1['cargo_motive'][0]:.4f}, Y:{r1['cargo_motive'][1]:.4f}, Z:{r1['cargo_motive'][2]:.4f}]")
    print(f"  AR 補正前（生）   : [X:{r1['cargo_ar_uncorr'][0]:.4f}, Y:{r1['cargo_ar_uncorr'][1]:.4f}, Z:{r1['cargo_ar_uncorr'][2]:.4f}]")
    print("\n【Run 2】")
    print(f"  Motive 真値座標   : [X:{r2['cargo_motive'][0]:.4f}, Y:{r2['cargo_motive'][1]:.4f}, Z:{r2['cargo_motive'][2]:.4f}]")
    print(f"  AR 補正前（生）   : [X:{r2['cargo_ar_uncorr'][0]:.4f}, Y:{r2['cargo_ar_uncorr'][1]:.4f}, Z:{r2['cargo_ar_uncorr'][2]:.4f}]")
    
    print("\n" + "="*60)
    print(" ■ 荷物の変位（Run 1 -> Run 2）の比較結果 [m]")
    print("="*60)
    print(f"  Motive真値変位 d_motive : [dX:{d_motive[0]:.4f}, dY:{d_motive[1]:.4f}, dZ:{d_motive[2]:.4f}] | 距離: {d_motive_len*100:.2f} cm")
    print(f"  AR補正前変位   d_uncorr : [dX:{d_ar_uncorr[0]:.4f}, dY:{d_ar_uncorr[1]:.4f}, dZ:{d_ar_uncorr[2]:.4f}] | 距離: {d_ar_uncorr_len*100:.2f} cm")
    print("-" * 60)
    print(f"  補正前変位誤差 (AR_raw - Motive)  : [eX:{err_uncorr[0]:.4f}, eY:{err_uncorr[1]:.4f}, eZ:{err_uncorr[2]:.4f}] | 距離誤差: {err_uncorr_len*100:.2f} cm")
    print("="*60)

    # 【方法3: 複数カメラ向きログによる CAMERA_OPTICAL_OFFSET 最小二乗キャリブレーション】
    calibrate_optical_offset(r1, r2, offset_cargo_r1, offset_hand_r1)

    if has_cam_r1 or has_cam_r2:
        print("\n" + "="*60)
        print(" ■ 精度向上用のカメラ座標系オフセット推奨値 (Motive真値 - AR生値) [m]")
        print("   ※ AR-hook1_test.py の該当のカメラオフセットに設定してください")
        print("="*60)
        if has_cam_r1:
            print("【Run 1 推奨値】")
            print(f"  手先 (HAND) オフセット:  [X:{offset_hand_r1[0]:.4f}, Y:{offset_hand_r1[1]:.4f}, Z:{offset_hand_r1[2]:.4f}]")
            print(f"  荷物 (CARGO) オフセット: [X:{offset_cargo_r1[0]:.4f}, Y:{offset_cargo_r1[1]:.4f}, Z:{offset_cargo_r1[2]:.4f}]")
        else:
            print("【Run 1】カメラ座標系データなし")
            
        if has_cam_r2:
            print("\n【Run 2 推奨値】")
            print(f"  手先 (HAND) オフセット:  [X:{offset_hand_r2[0]:.4f}, Y:{offset_hand_r2[1]:.4f}, Z:{offset_hand_r2[2]:.4f}]")
            print(f"  荷物 (CARGO) オフセット: [X:{offset_cargo_r2[0]:.4f}, Y:{offset_cargo_r2[1]:.4f}, Z:{offset_cargo_r2[2]:.4f}]")
        else:
            print("\n【Run 2】カメラ座標系データなし")
        print("="*60)

        # Run 1 のオフセットを Run 2 に適用した場合のシミュレーション評価
        if has_cam_r1 and has_cam_r2:
            # 荷物の補正
            cargo_r2_raw = np.array(r2['raw_data']['cargo_cam'])
            cargo_r2_motive = np.array(r2['raw_data']['motive_cargo_cam'])
            cargo_r2_corr = cargo_r2_raw + offset_cargo_r1
            
            raw_cargo_errs = np.linalg.norm(cargo_r2_raw - cargo_r2_motive, axis=1)
            corr_cargo_errs = np.linalg.norm(cargo_r2_corr - cargo_r2_motive, axis=1)
            
            # 手先の補正
            hand_r2_raw = np.array(r2['raw_data']['hand_cam'])
            hand_r2_motive = np.array(r2['raw_data']['motive_hand_cam'])
            hand_r2_corr = hand_r2_raw + offset_hand_r1
            
            raw_hand_errs = np.linalg.norm(hand_r2_raw - hand_r2_motive, axis=1)
            corr_hand_errs = np.linalg.norm(hand_r2_corr - hand_r2_motive, axis=1)
            
            print("\n" + "="*60)
            print(" ■ Run 1 のオフセットを Run 2 に適用した精度改善シミュレーション (カメラ座標系)")
            print("="*60)
            print("【荷物 (CARGO) 3D位置誤差】")
            print(f"  オフセット適用前 平均誤差: {np.mean(raw_cargo_errs)*100:.2f} cm")
            print(f"  オフセット適用後 平均誤差: {np.mean(corr_cargo_errs)*100:.2f} cm")
            print("\n【手先 (HAND) 3D位置誤差】")
            print(f"  オフセット適用前 平均誤差: {np.mean(raw_hand_errs)*100:.2f} cm")
            print(f"  オフセット適用後 平均誤差: {np.mean(corr_hand_errs)*100:.2f} cm")
            print("="*60)
    else:
        print("\nℹ️ CSVファイルにカメラ座標系データが含まれていないため、個別のオフセット算出およびシミュレーションはスキップされました。")
    
    # グラフ描画
    if HAS_MATPLOTLIB and HAS_NUMPY:
        plot_path = Path(run2_path).parent / "displacement_comparison.png"
        print(f"\n📊 グラフ画像を保存中: {plot_path}")
        
        plt.figure(figsize=(10, 8))
        
        # サンプル点プロット
        # Run 1
        m_c1 = np.array(r1['raw_data']['cargo_motive'])
        a_c1_raw = np.array(r1['raw_data']['cargo_ar_uncorr'])
        plt.scatter(m_c1[:, 0], m_c1[:, 1], color='red', alpha=0.1, label='Run 1: Motive Samples')
        plt.scatter(a_c1_raw[:, 0], a_c1_raw[:, 1], color='orange', alpha=0.1, label='Run 1: AR Raw Samples')
        
        # Run 2
        m_c2 = np.array(r2['raw_data']['cargo_motive'])
        a_c2_raw = np.array(r2['raw_data']['cargo_ar_uncorr'])
        plt.scatter(m_c2[:, 0], m_c2[:, 1], color='blue', alpha=0.1, label='Run 2: Motive Samples')
        plt.scatter(a_c2_raw[:, 0], a_c2_raw[:, 1], color='cyan', alpha=0.1, label='Run 2: AR Raw Samples')
        
        # 平均位置プロット
        plt.plot(r1['cargo_motive'][0], r1['cargo_motive'][1], 'ro', markersize=10, markeredgecolor='black', label='Run 1: Motive Mean')
        plt.plot(r1['cargo_ar_uncorr'][0], r1['cargo_ar_uncorr'][1], 's', color='darkorange', markersize=10, markeredgecolor='black', label='Run 1: AR Raw Mean')
        
        plt.plot(r2['cargo_motive'][0], r2['cargo_motive'][1], 'bo', markersize=10, markeredgecolor='black', label='Run 2: Motive Mean')
        plt.plot(r2['cargo_ar_uncorr'][0], r2['cargo_ar_uncorr'][1], 's', color='darkblue', markersize=10, markeredgecolor='black', label='Run 2: AR Raw Mean')
        
        # 変位ベクトルの描画（矢印）
        plt.quiver(r1['cargo_motive'][0], r1['cargo_motive'][1], d_motive[0], d_motive[1], 
                   angles='xy', scale_units='xy', scale=1, color='red', width=0.008, 
                   label=f'Motive Displacement Vector ({d_motive_len*100:.2f} cm)')
                   
        plt.quiver(r1['cargo_ar_uncorr'][0], r1['cargo_ar_uncorr'][1], d_ar_uncorr[0], d_ar_uncorr[1], 
                   angles='xy', scale_units='xy', scale=1, color='blue', width=0.008, 
                   label=f'AR Raw Vector ({d_ar_uncorr_len*100:.2f} cm)')
        
        plt.xlabel('X (East) Position [m]', fontsize=11)
        plt.ylabel('Y (North) Position [m]', fontsize=11)
        plt.title('Cargo Horizontal Position and Displacement Comparison\n(Motive Ground Truth vs Raw AR Camera)', fontsize=13, fontweight='bold')
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

