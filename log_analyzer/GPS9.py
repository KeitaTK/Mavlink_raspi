import os
import csv
from pymavlink import mavutil
import pyned2lla

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

    # ロールの位置情報を格納するリスト
    roll_positions = []

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
                # NED座標に変換
                x, y, z = pyned2lla.lla2ned(lat, lon, alt, ref_lat, ref_lon, ref_alt)
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
    if 'pyned2lla' in str(e):
        print("pip install pyned2lla")
    exit(1)
except Exception as e:
    print(f"エラーが発生しました: {e}")
    exit(1)
