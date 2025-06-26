from pymavlink import mavutil
import os

log_file = '/home/taki/Mavlink_raspi/log_analyzer/LOGS1/00000090.BIN'

if not os.path.exists(log_file):
    print(f"エラー: ログファイル {log_file} が見つかりません。パスを確認してください。")
    exit(1)

mlog = mavutil.mavlink_connection(log_file)
msg_types = set()
while True:
    msg = mlog.recv_match(blocking=False)
    if msg is None:
        break
    msg_types.add(msg.get_type())

print("ログファイルに含まれるメッセージタイプ:", sorted(msg_types))
