from pymavlink import mavutil
import csv
import os

# ログファイルのパス
log_file = '/home/taki/Mavlink_raspi/log_analyzer/bin/00000058.BIN'
# 出力CSVファイルのパス
output_csv = '/home/taki/Mavlink_raspi/log_analyzer/CSV/output3.csv'

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

    # データ格納用辞書
    gps_data = {}      # GPS生データ
    ekf_data = {}      # EKF推定位置データ
    guided_data = {}   # GUIDEDモードの目標位置データ

    # すべてのメッセージを読み込み、関連データをタイムスタンプで保存
    while True:
        msg = mlog.recv_match(blocking=False)
        if msg is None:
            break
        msg_type = msg.get_type()

        if msg_type == 'GPS':
            time_us = getattr(msg, 'TimeUS', None)
            if time_us is not None:
                gps_data[time_us] = {
                    'Lat': getattr(msg, 'Lat', None),
                    'Lng': getattr(msg, 'Lng', None),
                    'Alt': getattr(msg, 'Alt', None)
                }

        elif msg_type == 'POS':
            time_us = getattr(msg, 'TimeUS', None)
            if time_us is not None:
                ekf_data[time_us] = {
                    'Lat': getattr(msg, 'Lat', None),
                    'Lng': getattr(msg, 'Lng', None),
                    'Alt': getattr(msg, 'Alt', None)
                }

        elif msg_type == 'CMD':
            # MAV_CMD_NAV_WAYPOINT (16) または関連コマンドをフィルタリング
            if getattr(msg, 'CNum', None) == 16:
                time_us = getattr(msg, 'TimeUS', None)
                if time_us is not None:
                    guided_data[time_us] = {
                        'Lat': getattr(msg, 'Lat', None),
                        'Lng': getattr(msg, 'Lng', None),
                        'Alt': getattr(msg, 'Alt', None)
                    }

    # すべてのタイムスタンプを結合（GPS, EKF, GUIDEDのいずれかに存在するタイムスタンプ）
    all_times = sorted(set(gps_data.keys()) | set(ekf_data.keys()) | set(guided_data.keys()))

    # CSVに書き込み
    with open(output_csv, 'w', newline='') as f:
        writer = csv.writer(f)
        # ヘッダー行
        writer.writerow(['TimeUS', 'GPS_Lat', 'GPS_Lng', 'GPS_Alt', 'EKF_Lat', 'EKF_Lng', 'EKF_Alt', 'Guided_Lat', 'Guided_Lng', 'Guided_Alt'])
        # データ行
        for t in all_times:
            row = [t]
            gps = gps_data.get(t, {})
            ekf = ekf_data.get(t, {})
            guided = guided_data.get(t, {})
            row.extend([
                gps.get('Lat', None), gps.get('Lng', None), gps.get('Alt', None),
                ekf.get('Lat', None), ekf.get('Lng', None), ekf.get('Alt', None),
                guided.get('Lat', None), guided.get('Lng', None), guided.get('Alt', None)
            ])
            writer.writerow(row)

    print(f"結合されたCSVを {output_csv} に書き込みました。")
    print(f"記録されたデータ行数: {len(all_times)}")

except Exception as e:
    print(f"エラーが発生しました: {e}")
