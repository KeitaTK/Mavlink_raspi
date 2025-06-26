from pymavlink import mavutil
import csv
import os

# ログファイルのパス
log_file = '/home/taki/Mavlink_raspi/log_analyzer/LOGS1/00000091.BIN'
# 出力CSVファイルのパス
output_csv = '/home/taki/Mavlink_raspi/log_analyzer/CSV/merged_output2.csv'

# 出力ディレクトリが存在しない場合は作成
os.makedirs(os.path.dirname(output_csv), exist_ok=True)

# ログファイルの存在確認
if not os.path.exists(log_file):
    print(f"エラー: ログファイル {log_file} が見つかりません。パスを確認してください。")
    # 代替ディレクトリを検索
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

    # データ格納用辞書
    # key: TimeUS, value: dict with keys 'GPS', 'EKF', 'GUIDED' each holding tuple (Lat, Lng, Alt)
    data_dict = {}

    # すべてのメッセージを読み込み、関連データをタイムスタンプで保存
    while True:
        msg = mlog.recv_match(blocking=False)
        if msg is None:
            break
        msg_type = msg.get_type()

        if msg_type == 'GPS':
            time_us = getattr(msg, 'TimeUS', None)
            if time_us is not None:
                data_dict.setdefault(time_us, {})['GPS'] = (
                    getattr(msg, 'Lat', 0.0) or 0.0,
                    getattr(msg, 'Lng', 0.0) or 0.0,
                    getattr(msg, 'Alt', 0.0) or 0.0
                )

        elif msg_type == 'POS':
            time_us = getattr(msg, 'TimeUS', None)
            if time_us is not None:
                data_dict.setdefault(time_us, {})['EKF'] = (
                    getattr(msg, 'Lat', 0.0) or 0.0,
                    getattr(msg, 'Lng', 0.0) or 0.0,
                    getattr(msg, 'Alt', 0.0) or 0.0
                )

        elif msg_type == 'CMD':
            if getattr(msg, 'CNum', None) == 16:  # MAV_CMD_NAV_WAYPOINT
                time_us = getattr(msg, 'TimeUS', None)
                if time_us is not None:
                    data_dict.setdefault(time_us, {})['GUIDED'] = (
                        getattr(msg, 'Lat', 0.0) or 0.0,
                        getattr(msg, 'Lng', 0.0) or 0.0,
                        getattr(msg, 'Alt', 0.0) or 0.0
                    )

    # すべてのタイムスタンプをソート
    all_times = sorted(data_dict.keys())

    # CSVに書き込み
    with open(output_csv, 'w', newline='') as f:
        writer = csv.writer(f)
        # ヘッダー
        writer.writerow(['TimeUS', 'GPS_Lat', 'GPS_Lng', 'GPS_Alt', 'EKF_Lat', 'EKF_Lng', 'EKF_Alt', 'Guided_Lat', 'Guided_Lng', 'Guided_Alt'])
        
        # データ行
        for t in all_times:
            gps = data_dict[t].get('GPS', (0.0, 0.0, 0.0))
            ekf = data_dict[t].get('EKF', (0.0, 0.0, 0.0))
            guided = data_dict[t].get('GUIDED', (0.0, 0.0, 0.0))
            writer.writerow([
                t,
                gps[0], gps[1], gps[2],  # GPS: Lat, Lng, Alt
                ekf[0], ekf[1], ekf[2],  # EKF: Lat, Lng, Alt
                guided[0], guided[1], guided[2]  # GUIDED: Lat, Lng, Alt
            ])

    print(f"CSVファイルに書き込み完了: {output_csv}")
    print(f"記録された行数: {len(all_times)}")

except ImportError as e:
    print(f"エラー: pymavlinkがインストールされていません。以下のコマンドでインストールしてください。")
    print("pip install pymavlink")
    exit(1)
except Exception as e:
    print(f"エラーが発生しました: {e}")
    exit(1)
