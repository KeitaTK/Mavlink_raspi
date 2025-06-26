from pymavlink import mavutil
import csv
import os
import numpy as np
from pyproj import Transformer

# ログファイルのパス
log_file = '/home/taki/Mavlink_raspi/log_analyzer/LOGS1/00000090.BIN'
# 出力CSVファイルのパス
output_csv = '/home/taki/Mavlink_raspi/log_analyzer/CSV/merged_output7.csv'

# 基準GPS座標（7桁精度）
ref_lat = 36.0757800  # 緯度
ref_lon = 136.2132900  # 経度
ref_alt = 0.000       # 高度

# 出力ディレクトリが存在しない場合は作成
os.makedirs(os.path.dirname(output_csv), exist_ok=True)

# ログファイルの存在確認と探索
if not os.path.exists(log_file):
    print(f"エラー: ログファイル {log_file} が見つかりません。パスを確認してください。")
    # 代替ディレクトリを検索して .BIN ファイルを探す
    base_dirs = [
        '/home/taki/Mavlink_raspi/log_analyzer/LOGS1',
        '/home/taki/Mavlink_raspi/log_analyzer',
        '/home/taki/Mavlink_raspi',
        '/home/taki'
    ]
    bin_files = []
    for base_dir in base_dirs:
        if os.path.exists(base_dir):
            print(f"ディレクトリ {base_dir} を探索中...")
            try:
                for root, dirs, files in os.walk(base_dir):
                    for file in files:
                        if file.upper().endswith('.BIN'):
                            bin_files.append(os.path.join(root, file))
            except Exception as e:
                print(f"ディレクトリ {base_dir} の探索に失敗: {e}")
        else:
            print(f"ディレクトリ {base_dir} は存在しません。")
    
    if bin_files:
        print(f"見つかった.BINファイル: {bin_files}")
        log_file = bin_files[0]  # 最初のファイルを使用
        print(f"最初のファイルを使用します: {log_file}")
    else:
        print("ログファイルが見つかりませんでした。プログラムを終了します。")
        exit(1)

try:
    # ログを開く
    mlog = mavutil.mavlink_connection(log_file)
    print(f"ログファイル {log_file} を読み込みました。")

    # 開始点を特定するためのフラグ
    start_time = None
    gps_nonzero = False
    ekf_nonzero = False
    guided_nonzero = False

    # データ格納用リスト
    data_records = []

    # ECEFからNEDへの変換のための基準点のECEF座標を計算
    transformer_lla2ecef = Transformer.from_crs("EPSG:4326", "EPSG:4978")  # WGS84 LLA to ECEF
    ref_x, ref_y, ref_z = transformer_lla2ecef.transform(ref_lat, ref_lon, ref_alt)

    # すべてのメッセージを読み込み、データを抽出
    while True:
        msg = mlog.recv_match(blocking=False)
        if msg is None:
            break
        msg_type = msg.get_type()
        time_us = getattr(msg, 'TimeUS', None)

        if time_us is None:
            continue

        # GPSデータのチェック
        if msg_type == 'GPS':
            gps_lat = getattr(msg, 'Lat', 0.0) or 0.0
            gps_lng = getattr(msg, 'Lng', 0.0) or 0.0
            gps_alt = (getattr(msg, 'Alt', 0) or 0) / 100.0  # cm to m
            if (gps_lat != 0.0 or gps_lng != 0.0 or gps_alt != 0.0) and not gps_nonzero:
                gps_nonzero = True
        else:
            gps_lat = gps_lng = gps_alt = 0.0

        # EKFデータのチェック
        if msg_type == 'POS':
            ekf_lat = getattr(msg, 'Lat', 0.0) or 0.0
            ekf_lng = getattr(msg, 'Lng', 0.0) or 0.0
            ekf_alt = (getattr(msg, 'Alt', 0) or 0) / 100.0  # cm to m
            if (ekf_lat != 0.0 or ekf_lng != 0.0 or ekf_alt != 0.0) and not ekf_nonzero:
                ekf_nonzero = True
        else:
            ekf_lat = ekf_lng = ekf_alt = 0.0

        # GUIDEDデータのチェック
        if msg_type == 'GUIP':
            guided_x = getattr(msg, 'pX', 0.0) or 0.0
            guided_y = getattr(msg, 'pY', 0.0) or 0.0
            guided_z = getattr(msg, 'pZ', 0.0) or 0.0
            if (guided_x != 0.0 or guided_y != 0.0 or guided_z != 0.0) and not guided_nonzero:
                guided_nonzero = True
        else:
            guided_x = guided_y = guided_z = 0.0

        # 開始点の特定
        if start_time is None and gps_nonzero and ekf_nonzero and guided_nonzero:
            start_time = time_us
            print(f"開始点のTimeUS: {start_time}")

        # 開始点以降のデータを記録
        if start_time is not None:
            # GPSデータをNED座標に変換
            if gps_lat != 0.0 or gps_lng != 0.0 or gps_alt != 0.0:
                x_ecef, y_ecef, z_ecef = transformer_lla2ecef.transform(gps_lat, gps_lng, gps_alt)
                d_x = x_ecef - ref_x
                d_y = y_ecef - ref_y
                d_z = z_ecef - ref_z
                phi = np.radians(ref_lat)
                lam = np.radians(ref_lon)
                gps_x = -d_x * np.sin(lam) + d_y * np.cos(lam)  # East
                gps_y = -d_x * np.sin(phi) * np.cos(lam) - d_y * np.sin(phi) * np.sin(lam) + d_z * np.cos(phi)  # North
                gps_z = d_x * np.cos(phi) * np.cos(lam) + d_y * np.cos(phi) * np.sin(lam) + d_z * np.sin(phi)  # Down
            else:
                gps_x = gps_y = gps_z = 0.0

            # EKFデータをNED座標に変換
            if ekf_lat != 0.0 or ekf_lng != 0.0 or ekf_alt != 0.0:
                x_ecef, y_ecef, z_ecef = transformer_lla2ecef.transform(ekf_lat, ekf_lng, ekf_alt)
                d_x = x_ecef - ref_x
                d_y = y_ecef - ref_y
                d_z = z_ecef - ref_z
                phi = np.radians(ref_lat)
                lam = np.radians(ref_lon)
                ekf_x = -d_x * np.sin(lam) + d_y * np.cos(lam)  # East
                ekf_y = -d_x * np.sin(phi) * np.cos(lam) - d_y * np.sin(phi) * np.sin(lam) + d_z * np.cos(phi)  # North
                ekf_z = d_x * np.cos(phi) * np.cos(lam) + d_y * np.cos(phi) * np.sin(lam) + d_z * np.sin(phi)  # Down
            else:
                ekf_x = ekf_y = ekf_z = 0.0

            # GUIDEDデータはそのままローカル座標として扱う
            guided_x_local = guided_x if guided_x != 0.0 else 0.0
            guided_y_local = guided_y if guided_y != 0.0 else 0.0
            guided_z_local = guided_z if guided_z != 0.0 else 0.0

            data_records.append((
                time_us,
                gps_x, gps_y, gps_z,
                ekf_x, ekf_y, ekf_z,
                guided_x_local, guided_y_local, guided_z_local
            ))

    # CSVに書き込み
    with open(output_csv, 'w', newline='') as f:
        writer = csv.writer(f)
        # ヘッダー
        writer.writerow(['TimeUS', 'GPS_X', 'GPS_Y', 'GPS_Z', 'EKF_X', 'EKF_Y', 'EKF_Z', 'Guided_X', 'Guided_Y', 'Guided_Z'])
        for row in data_records:
            writer.writerow(row)

    print(f"ローカル座標データをCSVに書き込みました: {output_csv}")
    print(f"記録された行数: {len(data_records)}")

except ImportError as e:
    print(f"エラー: 必要なライブラリがインストールされていません。以下のコマンドでインストールしてください。")
    if 'pymavlink' in str(e):
        print("pip install pymavlink")
    if 'pyproj' in str(e):
        print("pip install pyproj")
    if 'numpy' in str(e):
        print("pip install numpy")
    exit(1)
except Exception as e:
    print(f"エラーが発生しました: {e}")
    exit(1)
