from pymavlink import mavutil
import csv
import os
import numpy as np
from pyproj import Transformer

# ログファイルのパス
log_file = '/home/taki/Mavlink_raspi/log_analyzer/LOGS1/00000090.BIN'
output_csv = '/home/taki/Mavlink_raspi/log_analyzer/CSV/merged_output6_complemented.csv'

# 基準座標
ref_lat = 36.0757800
ref_lon = 136.2132900
ref_alt = 0.0

os.makedirs(os.path.dirname(output_csv), exist_ok=True)

transformer_lla2ecef = Transformer.from_crs("EPSG:4326", "EPSG:4978")
ref_x, ref_y, ref_z = transformer_lla2ecef.transform(ref_lat, ref_lon, ref_alt)

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

    # GPS
    if msg_type == 'GPS':
        gps_lat = getattr(msg, 'Lat', 0.0) or 0.0
        gps_lng = getattr(msg, 'Lng', 0.0) or 0.0
        gps_alt = (getattr(msg, 'Alt', 0) or 0) / 100.0
    else:
        gps_lat = gps_lng = gps_alt = 0.0

    # EKF
    if msg_type == 'POS':
        ekf_lat = getattr(msg, 'Lat', 0.0) or 0.0
        ekf_lng = getattr(msg, 'Lng', 0.0) or 0.0
        ekf_alt = (getattr(msg, 'Alt', 0) or 0) / 100.0
    else:
        ekf_lat = ekf_lng = ekf_alt = 0.0

    # Guided - GPS座標として解釈してNED変換
    if msg_type == 'GUIP':
        guided_px = getattr(msg, 'pX', 0.0) or 0.0
        guided_py = getattr(msg, 'pY', 0.0) or 0.0
        guided_pz = getattr(msg, 'pZ', 0.0) or 0.0
        
        # pX, pYがGPS座標（スケールされている可能性）として解釈
        if guided_px != 0.0 and guided_py != 0.0:
            # 値が大きすぎる場合（1e7倍されている場合）はスケールダウン
            if abs(guided_px) > 1000:  # 1000を超える場合は度ではなく1e7倍された値
                guided_lat = guided_px / 1e7
                guided_lng = guided_py / 1e7
            else:
                guided_lat = guided_px
                guided_lng = guided_py
            
            # pZも高度として扱う（メートル単位と仮定）
            guided_alt = guided_pz
            
            # GPS座標からNED座標に変換
            x_ecef, y_ecef, z_ecef = transformer_lla2ecef.transform(guided_lat, guided_lng, guided_alt)
            d_x = x_ecef - ref_x
            d_y = y_ecef - ref_y
            d_z = z_ecef - ref_z
            phi = np.radians(ref_lat)
            lam = np.radians(ref_lon)
            guided_x = -d_x * np.sin(lam) + d_y * np.cos(lam)
            guided_y = -d_x * np.sin(phi) * np.cos(lam) - d_y * np.sin(phi) * np.sin(lam) + d_z * np.cos(phi)
            guided_z = d_x * np.cos(phi) * np.cos(lam) + d_y * np.cos(phi) * np.sin(lam) + d_z * np.sin(phi)
        else:
            guided_x = guided_y = guided_z = 0.0
    else:
        guided_x = guided_y = guided_z = 0.0

    # GPS->NED
    if gps_lat != 0.0 or gps_lng != 0.0 or gps_alt != 0.0:
        x_ecef, y_ecef, z_ecef = transformer_lla2ecef.transform(gps_lat, gps_lng, gps_alt)
        d_x = x_ecef - ref_x
        d_y = y_ecef - ref_y
        d_z = z_ecef - ref_z
        phi = np.radians(ref_lat)
        lam = np.radians(ref_lon)
        gps_x = -d_x * np.sin(lam) + d_y * np.cos(lam)
        gps_y = -d_x * np.sin(phi) * np.cos(lam) - d_y * np.sin(phi) * np.sin(lam) + d_z * np.cos(phi)
        gps_z = d_x * np.cos(phi) * np.cos(lam) + d_y * np.cos(phi) * np.sin(lam) + d_z * np.sin(phi)
    else:
        gps_x = gps_y = gps_z = 0.0

    # EKF->NED
    if ekf_lat != 0.0 or ekf_lng != 0.0 or ekf_alt != 0.0:
        x_ecef, y_ecef, z_ecef = transformer_lla2ecef.transform(ekf_lat, ekf_lng, ekf_alt)
        d_x = x_ecef - ref_x
        d_y = y_ecef - ref_y
        d_z = z_ecef - ref_z
        phi = np.radians(ref_lat)
        lam = np.radians(ref_lon)
        ekf_x = -d_x * np.sin(lam) + d_y * np.cos(lam)
        ekf_y = -d_x * np.sin(phi) * np.cos(lam) - d_y * np.sin(phi) * np.sin(lam) + d_z * np.cos(phi)
        ekf_z = d_x * np.cos(phi) * np.cos(lam) + d_y * np.cos(phi) * np.sin(lam) + d_z * np.sin(phi)
    else:
        ekf_x = ekf_y = ekf_z = 0.0

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

print(f"補完＆不要行削除後のローカル座標データをCSVに書き込みました: {output_csv}")
print(f"記録された行数: {len(data_records[start_idx:])}")
