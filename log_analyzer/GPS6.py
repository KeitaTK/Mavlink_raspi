from pymavlink import mavutil
import csv
import os

# ログファイルのパス（初期値）
log_file = '/home/taki/Mavlink_raspi/log_analyzer/LOGS1/00000091.BIN'
# 出力CSVファイルのパス
output_csv = '/home/taki/Mavlink_raspi/log_analyzer/CSV/merged_output3.csv'

# 出力ディレクトリが存在しない場合は作成
os.makedirs(os.path.dirname(output_csv), exist_ok=True)

# ログファイルの存在確認と探索
if not os.path.exists(log_file):
    print(f"エラー: ログファイル {log_file} が見つかりません。パスを確認してください。")
    # 代替ディレクトリを検索して .BIN ファイルを探す
    base_dirs = [
        '/home/taki/Mavlink_raspi/log_analyzer/bin',
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

        elif msg_type == 'GUIP':
            time_us = getattr(msg, 'TimeUS', None)
            if time_us is not None:
                data_dict.setdefault(time_us, {})['GUIDED'] = (
                    getattr(msg, 'pX', 0.0) or 0.0,  # GUIPメッセージのpX, pY, pZを使用
                    getattr(msg, 'pY', 0.0) or 0.0,
                    getattr(msg, 'pZ', 0.0) or 0.0
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
                guided[0], guided[1], guided[2]  # GUIDED: Lat, Lng, Alt (from GUIP)
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
