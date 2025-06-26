from pymavlink import mavutil
import csv
import os

# ログファイルのパス（初期値）
log_file = '/home/taki/Mavlink_raspi/log_analyzer/LOGS1/00000090.BIN'
# 出力CSVファイルのパス
output_csv = '/home/taki/Mavlink_raspi/log_analyzer/CSV/merged_output4.csv'

# 出力ディレクトリが存在しない場合は作成
os.makedirs(os.path.dirname(output_csv), exist_ok=True)

if not os.path.exists(log_file):
    print(f"エラー: ログファイル {log_file} が見つかりません。")
    exit(1)

try:
    mlog = mavutil.mavlink_connection(log_file)
    print(f"ログファイル {log_file} を読み込みました。")

    rows = [['TimeUS', 'pX', 'pY', 'pZ']]
    while True:
        msg = mlog.recv_match(type='GUIP', blocking=False)
        if msg is None:
            break
        rows.append([
            getattr(msg, 'TimeUS', 0),
            getattr(msg, 'pX', 0),
            getattr(msg, 'pY', 0),
            getattr(msg, 'pZ', 0)
        ])

    with open(output_csv, 'w', newline='') as f:
        csv.writer(f).writerows(rows)

    print(f"GUIPのpX,pY,pZをCSVに書き込みました: {output_csv}")
    print(f"記録されたデータ行数: {len(rows)-1}")

except Exception as e:
    print(f"エラーが発生しました: {e}")
