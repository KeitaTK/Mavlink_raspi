from pymavlink import mavutil
import csv
import os

# ログファイルのパス（検索結果に基づき、存在するパスを仮定）
log_file = '/home/taki/Mavlink_raspi/log_analyzer/LOGS1/00000090.BIN'
# 出力CSVファイルのパス
output_csv = '/home/taki/Mavlink_raspi/log_analyzer/CSV/extracted_data1.csv'

# 出力ディレクトリが存在しない場合は作成
os.makedirs(os.path.dirname(output_csv), exist_ok=True)

# ログファイルの存在確認
if not os.path.exists(log_file):
    print(f"エラー: ログファイル {log_file} が見つかりません。パスを確認してください。")
    # 代替パスを試す（検索結果に基づき、ディレクトリ構造を確認）
    possible_dirs = [
        '/home/taki/Mavlink_raspi/log_analyzer/bin',
        '/home/taki/Mavlink_raspi/log_analyzer',
        '/home/taki/Mavlink_raspi'
    ]
    for dir_path in possible_dirs:
        if os.path.exists(dir_path):
            print(f"ディレクトリ {dir_path} が見つかりました。内容を確認してください。")
            try:
                files = os.listdir(dir_path)
                print(f"内容: {files}")
            except Exception as e:
                print(f"ディレクトリ {dir_path} の内容確認に失敗: {e}")
        else:
            print(f"ディレクトリ {dir_path} は存在しません。")
    exit(1)

try:
    # ログを開く
    mlog = mavutil.mavlink_connection(log_file)
    print(f"ログファイル {log_file} を読み込みました。")

    # データ格納用リスト
    rows = []
    rows.append(['TimeUS', 'Type', 'Lat', 'Lng', 'Alt'])

    # すべてのメッセージを読み込み、関連データを抽出
    while True:
        msg = mlog.recv_match(blocking=False)
        if msg is None:
            break
        msg_type = msg.get_type()

        if msg_type == 'GPS':
            time_us = getattr(msg, 'TimeUS', None)
            if time_us is not None:
                rows.append([
                    time_us,
                    'GPS',
                    getattr(msg, 'Lat', None),
                    getattr(msg, 'Lng', None),
                    getattr(msg, 'Alt', None)
                ])

        elif msg_type == 'POS':
            time_us = getattr(msg, 'TimeUS', None)
            if time_us is not None:
                rows.append([
                    time_us,
                    'EKF',
                    getattr(msg, 'Lat', None),
                    getattr(msg, 'Lng', None),
                    getattr(msg, 'Alt', None)
                ])

        elif msg_type == 'CMD':
            if getattr(msg, 'CNum', None) == 16:  # MAV_CMD_NAV_WAYPOINT
                time_us = getattr(msg, 'TimeUS', None)
                if time_us is not None:
                    rows.append([
                        time_us,
                        'GUIDED',
                        getattr(msg, 'Lat', None),
                        getattr(msg, 'Lng', None),
                        getattr(msg, 'Alt', None)
                    ])

    # CSVに書き込み
    with open(output_csv, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerows(rows)

    print(f"データをCSVに書き込みました: {output_csv}")
    print(f"記録されたデータ行数: {len(rows) - 1}")  # ヘッダー行を除く

except Exception as e:
    print(f"エラーが発生しました: {e}")
