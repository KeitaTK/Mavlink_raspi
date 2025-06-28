# 同じディレクトリに入れたbinのログファイルからGPS,EKF,guidedモードの目標を取得してCSVにする

from pymavlink import mavutil
import csv
import os
import numpy as np
from pyproj import Transformer

# ログファイルのパス
log_file = r'C:\Users\KT\Documents\local\Mavlink_raspi\log_analyzer\LOGS1\00000107.BIN'
output_csv = r"C:\Users\KT\Documents\local\Mavlink_raspi\log_analyzer\CSV\local_13.csv"

# 基準座標（ローカル座標系の原点）
ref_lat = 36.0757800
ref_lon = 136.2132900
ref_alt = 0.0

# 出力ディレクトリが存在しない場合は作成
os.makedirs(os.path.dirname(output_csv), exist_ok=True)

# 座標変換のためのTransformer
transformer_lla2ecef = Transformer.from_crs("EPSG:4326", "EPSG:4978")
ref_x, ref_y, ref_z = transformer_lla2ecef.transform(ref_lat, ref_lon, ref_alt)

def lla_to_ned(lat, lon, alt):
    """緯度経度高度をNED座標に変換"""
    if lat == 0.0 and lon == 0.0 and alt == 0.0:
        return 0.0, 0.0, 0.0
    
    x_ecef, y_ecef, z_ecef = transformer_lla2ecef.transform(lat, lon, alt)
    d_x = x_ecef - ref_x
    d_y = y_ecef - ref_y
    d_z = z_ecef - ref_z
    
    phi = np.radians(ref_lat)
    lam = np.radians(ref_lon)
    
    # 正しいNED変換（North-East-Down順序に修正）
    ned_north = -d_x * np.sin(phi) * np.cos(lam) - d_y * np.sin(phi) * np.sin(lam) + d_z * np.cos(phi)
    ned_east = -d_x * np.sin(lam) + d_y * np.cos(lam)
    ned_down = -d_x * np.cos(phi) * np.cos(lam) - d_y * np.cos(phi) * np.sin(lam) - d_z * np.sin(phi)
    
    return ned_north, ned_east, ned_down

mlog = mavutil.mavlink_connection(log_file)
print(f"ログファイル {log_file} を読み込みました。")

data_records = []

while True:
    msg = mlog.recv_match(blocking=False)
    if msg is None:
        break
    msg_type = msg.get_type()
    time_us = getattr(msg, 'TimeUS', None)
    if time_us is None:
        continue

    # GPS - 度の単位で記録されている、既にメートル単位
    if msg_type == 'GPS':
        gps_lat = getattr(msg, 'Lat', 0.0) or 0.0
        gps_lng = getattr(msg, 'Lng', 0.0) or 0.0
        gps_alt = getattr(msg, 'Alt', 0.0) or 0.0  # 既にメートル単位
        # NED座標に変換（修正版）
        gps_x, gps_y, gps_z = lla_to_ned(gps_lat, gps_lng, gps_alt)
    else:
        gps_x = gps_y = gps_z = 0.0

    # EKF - 度の単位で記録されている、既にメートル単位
    if msg_type == 'POS':
        ekf_lat = getattr(msg, 'Lat', 0.0) or 0.0
        ekf_lng = getattr(msg, 'Lng', 0.0) or 0.0
        ekf_alt = getattr(msg, 'Alt', 0.0) or 0.0  # 既にメートル単位
        # NED座標に変換（修正版）
        ekf_x, ekf_y, ekf_z = lla_to_ned(ekf_lat, ekf_lng, ekf_alt)
    else:
        ekf_x = ekf_y = ekf_z = 0.0

    # Guided - pX, pYが1e7倍された緯度経度、pZが高度（cm単位と仮定）
    if msg_type == 'GUIP':
        guided_px_raw = getattr(msg, 'pX', 0.0) or 0.0
        guided_py_raw = getattr(msg, 'pY', 0.0) or 0.0
        guided_pz_raw = getattr(msg, 'pZ', 0.0) or 0.0
        
        if guided_px_raw != 0.0 and guided_py_raw != 0.0:
            # 1e7倍された値を度に戻す
            guided_lat = guided_px_raw / 1e7
            guided_lng = guided_py_raw / 1e7
            guided_alt_cm = guided_pz_raw  # センチメートル単位と仮定
            guided_alt = guided_alt_cm / 100.0  # メートルに変換
            # NED座標に変換（修正版）
            guided_x, guided_y, guided_z = lla_to_ned(guided_lat, guided_lng, guided_alt)
        else:
            guided_x = guided_y = guided_z = 0.0
    else:
        guided_x = guided_y = guided_z = 0.0

    data_records.append([
        time_us,
        gps_x, gps_y, gps_z,
        ekf_x, ekf_y, ekf_z,
        guided_x, guided_y, guided_z
    ])

# 0値を補完
def complement_cols(records, cols):
    last = [0.0]*cols
    for row in records:
        for i in range(cols):
            if row[i+1] == 0.0 and last[i] != 0.0:
                row[i+1] = last[i]
            elif row[i+1] != 0.0:
                last[i] = row[i+1]
    return records

data_records = complement_cols(data_records, 9)

# 最初に全てが0でない行を探す
start_idx = 0
for i, row in enumerate(data_records):
    if all(row[j] != 0.0 for j in range(1, 10)):
        start_idx = i
        break

# その行以降だけを出力
with open(output_csv, 'w', newline='') as f:
    writer = csv.writer(f)
    writer.writerow(['TimeUS', 'GPS_X', 'GPS_Y', 'GPS_Z', 'EKF_X', 'EKF_Y', 'EKF_Z', 'Guided_X', 'Guided_Y', 'Guided_Z'])
    for row in data_records[start_idx:]:
        writer.writerow(row)

print(f"ローカル座標系データをCSVに書き込みました: {output_csv}")
print(f"記録された行数: {len(data_records[start_idx:])}")

# データの範囲を表示（検証用）
if data_records:
    print(f"\n=== ローカル座標系データの範囲確認 ===")
    print(f"基準座標: 緯度={ref_lat}, 経度={ref_lon}, 高度={ref_alt}")
    
    for start_idx_check in range(min(len(data_records), start_idx + 10)):
        if start_idx_check >= start_idx:
            row = data_records[start_idx_check]
            print(f"時刻 {row[0]}: GPS({row[1]:.2f}, {row[2]:.2f}, {row[3]:.2f}) "
                  f"EKF({row[4]:.2f}, {row[5]:.2f}, {row[6]:.2f}) "
                  f"Guided({row[7]:.2f}, {row[8]:.2f}, {row[9]:.2f})")
            if start_idx_check >= start_idx + 5:  # 最初の5行だけ表示
                break
