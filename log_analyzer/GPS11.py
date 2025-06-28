from pymavlink import mavutil
import csv
import os

# ログファイルのパス
log_file = '/home/taki/Mavlink_raspi/log_analyzer/LOGS1/00000090.BIN'
output_csv = '/home/taki/Mavlink_raspi/log_analyzer/CSV/raw1.csv'

# 出力ディレクトリが存在しない場合は作成
os.makedirs(os.path.dirname(output_csv), exist_ok=True)

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

    # GPS - 生の値をそのまま記録
    if msg_type == 'GPS':
        gps_lat = getattr(msg, 'Lat', 0.0) or 0.0
        gps_lng = getattr(msg, 'Lng', 0.0) or 0.0
        gps_alt_raw = getattr(msg, 'Alt', 0) or 0  # 生の値（通常はcm単位）
    else:
        gps_lat = gps_lng = gps_alt_raw = 0.0

    # EKF - 生の値をそのまま記録
    if msg_type == 'POS':
        ekf_lat = getattr(msg, 'Lat', 0.0) or 0.0
        ekf_lng = getattr(msg, 'Lng', 0.0) or 0.0
        ekf_alt_raw = getattr(msg, 'Alt', 0) or 0  # 生の値（通常はcm単位）
    else:
        ekf_lat = ekf_lng = ekf_alt_raw = 0.0

    # Guided - 生の値をそのまま記録（スケーリング処理なし）
    if msg_type == 'GUIP':
        guided_px_raw = getattr(msg, 'pX', 0.0) or 0.0  # 生の値
        guided_py_raw = getattr(msg, 'pY', 0.0) or 0.0  # 生の値
        guided_pz_raw = getattr(msg, 'pZ', 0.0) or 0.0  # 生の値
    else:
        guided_px_raw = guided_py_raw = guided_pz_raw = 0.0

    data_records.append([
        time_us,
        gps_lat, gps_lng, gps_alt_raw,
        ekf_lat, ekf_lng, ekf_alt_raw,
        guided_px_raw, guided_py_raw, guided_pz_raw
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
    writer.writerow(['TimeUS', 'GPS_Lat', 'GPS_Lng', 'GPS_Alt_Raw', 'EKF_Lat', 'EKF_Lng', 'EKF_Alt_Raw', 'Guided_pX_Raw', 'Guided_pY_Raw', 'Guided_pZ_Raw'])
    for row in data_records[start_idx:]:
        writer.writerow(row)

print(f"生データをCSVに書き込みました: {output_csv}")
print(f"記録された行数: {len(data_records[start_idx:])}")

# データの範囲を表示（検証用）
if data_records:
    print("\n=== データ範囲の確認 ===")
    gps_lats = [row[1] for row in data_records[start_idx:] if row[1] != 0.0]
    gps_lngs = [row[2] for row in data_records[start_idx:] if row[2] != 0.0]
    gps_alts = [row[3] for row in data_records[start_idx:] if row[3] != 0.0]
    
    ekf_lats = [row[4] for row in data_records[start_idx:] if row[4] != 0.0]
    ekf_lngs = [row[5] for row in data_records[start_idx:] if row[5] != 0.0]
    ekf_alts = [row[6] for row in data_records[start_idx:] if row[6] != 0.0]
    
    guided_pxs = [row[7] for row in data_records[start_idx:] if row[7] != 0.0]
    guided_pys = [row[8] for row in data_records[start_idx:] if row[8] != 0.0]
    guided_pzs = [row[9] for row in data_records[start_idx:] if row[9] != 0.0]
    
    if gps_lats:
        print(f"GPS緯度範囲: {min(gps_lats):.7f} ~ {max(gps_lats):.7f}")
    if gps_lngs:
        print(f"GPS経度範囲: {min(gps_lngs):.7f} ~ {max(gps_lngs):.7f}")
    if gps_alts:
        print(f"GPS高度（生値）範囲: {min(gps_alts)} ~ {max(gps_alts)}")
    
    if ekf_lats:
        print(f"EKF緯度範囲: {min(ekf_lats):.7f} ~ {max(ekf_lats):.7f}")
    if ekf_lngs:
        print(f"EKF経度範囲: {min(ekf_lngs):.7f} ~ {max(ekf_lngs):.7f}")
    if ekf_alts:
        print(f"EKF高度（生値）範囲: {min(ekf_alts)} ~ {max(ekf_alts)}")
    
    if guided_pxs:
        print(f"Guided pX（生値）範囲: {min(guided_pxs)} ~ {max(guided_pxs)}")
    if guided_pys:
        print(f"Guided pY（生値）範囲: {min(guided_pys)} ~ {max(guided_pys)}")
    if guided_pzs:
        print(f"Guided pZ（生値）範囲: {min(guided_pzs)} ~ {max(guided_pzs)}")
