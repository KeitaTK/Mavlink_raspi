from pymavlink import mavutil
import csv
import os

# ログファイルのパス（指定されたディレクトリに基づく）
log_file = '/home/taki/Mavlink_raspi/log_analyzer/bin/00000058.BIN'
# 出力CSVファイルのパス
output_csv = '/home/taki/Mavlink_raspi/log_analyzer/CSV/output2.csv'

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

    # データ格納用リスト
    rows = []
    rows.append(['TimeUS', 'GPS_Alt', 'EKF_Lat', 'EKF_Lon', 'EKF_Alt'])

    # 一時保存用
    gps_alt = None
    gps_time = None
    ekf_lat = None
    ekf_lon = None
    ekf_alt = None
    ekf_time = None

    while True:
        msg = mlog.recv_match(blocking=False)
        if msg is None:
            break

        # GPS生データの高度取得
        if msg.get_type() == 'GPS':
            gps_time = getattr(msg, 'TimeUS', None)
            gps_alt = getattr(msg, 'Alt', None)  # 単位はcmやmmの場合があるので要確認

        # EKF推定後の位置情報取得（POSメッセージを使用）
        if msg.get_type() == 'POS':
            ekf_time = getattr(msg, 'TimeUS', None)
            ekf_lat = getattr(msg, 'Lat', None)   # 緯度
            ekf_lon = getattr(msg, 'Lng', None)   # 経度
            ekf_alt = getattr(msg, 'Alt', None)   # 高度

            # GPSの値とタイムスタンプが近い場合だけ記録（例：500ms以内に緩和）
            if gps_time is not None and abs((ekf_time if ekf_time else 0) - (gps_time if gps_time else 0)) < 500000:
                rows.append([
                    gps_time,
                    gps_alt,
                    ekf_lat if ekf_lat else None,
                    ekf_lon if ekf_lon else None,
                    ekf_alt if ekf_alt else None
                ])

    # CSVに書き込み
    with open(output_csv, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerows(rows)

    print(f'書き出し完了: {output_csv}')
    print(f'記録されたデータ行数: {len(rows) - 1}')  # ヘッダー行を除く

except Exception as e:
    print(f"エラーが発生しました: {e}")
