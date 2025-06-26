from pymavlink import mavutil
import csv
import os
import numpy as np
from pyproj import Transformer

# ログファイルのパス（初期値）
log_file = '/home/taki/Mavlink_raspi/log_analyzer/LOGS1/00000090.BIN'
# 出力CSVファイルのパス
output_csv = '/home/taki/Mavlink_raspi/log_analyzer/CSV/merged_output6.csv'


# 基準GPS座標（7桁精度）
ref_lat = 36.0757800  # 緯度
ref_lon = 136.2132900  # 経度
ref_alt = 0.000       # 高度

# 出力ディレクトリが存在しない場合は作成
os.makedirs(os.path.dirname(output_csv), exist_ok=True)

# ログファイルの存在確認
if not os.path.exists(log_file):
    print(f"エラー: ログファイル {log_file} が見つかりません。パスを確認してください。")
    exit(1)

try:
    # ログを開く
    mlog = mavutil.mavlink_connection(log_file)
    print(f"ログファイル {log_file} を読み込みました。")

    # ロールの位置情報を格納するリスト
    roll_positions = []

    # ECEFからNEDへの変換のための基準点のECEF座標を計算
    transformer_lla2ecef = Transformer.from_crs("EPSG:4326", "EPSG:4978")  # WGS84 LLA to ECEF
    ref_x, ref_y, ref_z = transformer_lla2ecef.transform(ref_lat, ref_lon, ref_alt)

    # すべてのメッセージを読み込み、GPSデータを抽出
    while True:
        msg = mlog.recv_match(blocking=False)
        if msg is None:
            break
        msg_type = msg.get_type()

        if msg_type == 'GPS':
            time_us = getattr(msg, 'TimeUS', None)
            if time_us is not None:
                lat = getattr(msg, 'Lat', 0.0)
                lon = getattr(msg, 'Lng', 0.0)
                alt_cm = getattr(msg, 'Alt', 0)  # センチメートル単位
                alt = alt_cm / 100.0  # メートルに変換
                
                # LLAからECEFに変換
                x_ecef, y_ecef, z_ecef = transformer_lla2ecef.transform(lat, lon, alt)
                
                # ECEFからNEDに変換
                d_x = x_ecef - ref_x
                d_y = y_ecef - ref_y
                d_z = z_ecef - ref_z
                
                # 基準点の緯度・経度をラジアンに変換
                phi = np.radians(ref_lat)
                lam = np.radians(ref_lon)
                
                # ECEF差分をNEDに変換
                x = -d_x * np.sin(lam) + d_y * np.cos(lam)  # East
                y = -d_x * np.sin(phi) * np.cos(lam) - d_y * np.sin(phi) * np.sin(lam) + d_z * np.cos(phi)  # North
                z = d_x * np.cos(phi) * np.cos(lam) + d_y * np.cos(phi) * np.sin(lam) + d_z * np.sin(phi)  # Down
                
                # ロールの位置情報としてNED座標を保存（単位：メートル）
                roll_positions.append((time_us, x, y, z))

    # CSVに書き込み
    with open(output_csv, 'w', newline='') as f:
        writer = csv.writer(f)
        # ヘッダー
        writer.writerow(['TimeUS', 'Roll_X', 'Roll_Y', 'Roll_Z'])
        for row in roll_positions:
            writer.writerow(row)

    print(f"ロールの位置情報をNED座標でCSVに書き込みました: {output_csv}")
    print(f"記録された行数: {len(roll_positions)}")

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
