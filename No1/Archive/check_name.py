#!/usr/bin/env python3
from pymavlink import mavutil

def connect():
    """USB または UART 接続を選択して MAVLink 接続を確立"""
    if input("USB接続を行いますか (y/n): ").strip().lower() == 'y':
        return mavutil.mavlink_connection('/dev/ttyACM0', baud=115200)
    else:
        return mavutil.mavlink_connection('/dev/ttyAMA0', baud=1000000, rtscts=True)

def wait_heartbeat(master):
    print("接続を待機中...")
    master.wait_heartbeat()
    print(f"接続完了 (システム: {master.target_system}, コンポーネント: {master.target_component})")

def list_param_names(master):
    """全パラメータ名を取得して名前のみを表示"""
    print("全パラメータ名をリクエスト中…")
    master.mav.param_request_list_send(
        master.target_system,
        master.target_component
    )
    print("パラメータ名を受信中…Ctrl+Cで終了")
    try:
        while True:
            msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
            if not msg:
                continue
            # msg.param_id は既に文字列なのでそのまま扱う
            name = msg.param_id.rstrip('\x00')
            print(name)
    except KeyboardInterrupt:
        print("パラメータ名取得を終了")

def main():
    master = connect()
    wait_heartbeat(master)
    list_param_names(master)

if __name__ == "__main__":
    main()
