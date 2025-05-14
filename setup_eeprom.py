# パラメータをEEPROMに永続的に保存するコード
# 最初にインポートと接続が必要です
from pymavlink import mavutil
import time

# 接続を確立（実際の接続方法に合わせて変更してください）
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)

# ハートビートを待機（接続確認）
master.wait_heartbeat()
print(f"接続完了 (システム: {master.target_system}, コンポーネント: {master.target_component})")

# EEPROMへの永続保存コマンドを送信
print("設定をEEPROMに永続保存中...")
master.mav.command_long_send(
    master.target_system,
    master.target_component,
    mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE,
    0,  # 確認パラメータ
    1,  # 1=Write parameters（設定を保存）
    0, 0, 0, 0, 0, 0  # 未使用のパラメータ
)

# コマンドACKを待機（重要）
ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=10)
if ack and ack.command == mavutil.mavlink.MAV_CMD_PREFLIGHT_STORAGE and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
    print("EEPROMへの保存成功")
else:
    print("EEPROMへの保存でエラーまたはタイムアウトが発生")
