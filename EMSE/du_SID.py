# -*- coding: utf-8 -*-

from pymavlink import mavutil
import time

# --- 設定項目 ---
# 使用するシリアルポートに合わせて変更してください
# Raspberry Pi 3/4/5: '/dev/serial0' or '/dev/ttyAMA0'
# USB接続: '/dev/ttyACM0' など
CONNECTION_STRING = '/dev/ttyAMA0'
BAUD_RATE = 1000000

# ArduCopterのフライトモード番号
# https://ardupilot.org/copter/docs/parameters.html#fltmode1
STABILIZE_MODE = 0
SYSTEM_ID_MODE = 25

def main():
    """
    メイン処理
    """
    # フライトコントローラーへの接続
    master = None
    try:
        print(f"Connecting to flight controller on {CONNECTION_STRING} at {BAUD_RATE} baud...")
        master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD_RATE, rtscts=True)
    except Exception as e:
        print(f"Error connecting: {e}")
        return

    # Heartbeatを待つ
    try:
        master.wait_heartbeat(timeout=5)
        print(f"Heartbeat received from system {master.target_system}, component {master.target_component}")
    except Exception as e:
        print(f"Timeout waiting for heartbeat: {e}")
        master.close()
        return

    # モード変更が完了したかどうかのフラグ
    mode_change_requested = False

    print("--- Stabilizeモードで飛行を開始してください ---")
    print("スクリプトは現在のモードを監視し、System IDへの切り替えを試みます。")

    try:
        while True:
            # Heartbeatメッセージを受信して現在のモードを確認
            msg = master.recv_match(type='HEARTBEAT', blocking=True, timeout=3)
            if not msg:
                print("No heartbeat received for 3 seconds. Exiting.")
                break
            
            current_mode = msg.custom_mode
            print(f"Current Mode ID: {current_mode}")

            # まだモード変更をリクエストしておらず、現在のモードがStabilizeの場合
            if not mode_change_requested and current_mode == STABILIZE_MODE:
                print("Stabilize mode detected. Requesting switch to System ID mode...")
                
                # モード変更コマンドを送信
                master.mav.command_long_send(
                    master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                    0,  # confirmation
                    mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
                    SYSTEM_ID_MODE,
                    0, 0, 0, 0, 0  # unused params
                )
                
                # コマンドに対する応答(COMMAND_ACK)を待つ
                ack = master.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
                
                if not ack:
                    print("Error: No COMMAND_ACK received from flight controller.")
                elif ack.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                    print("Success: Flight controller accepted the mode change command.")
                    print("Waiting for mode to be confirmed via Heartbeat...")
                else:
                    print(f"Error: Mode change command was rejected. Result code: {ack.result}")
                
                # コマンドを再送しないようにフラグを立てる
                mode_change_requested = True

            # モードがSystem IDに切り替わったら、プログラムを終了
            if current_mode == SYSTEM_ID_MODE:
                print("Mode successfully changed to System ID. Exiting script.")
                break

            time.sleep(1) # 1秒ごとにモードをチェック

    except KeyboardInterrupt:
        print("Script terminated by user.")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        if master:
            master.close()
            print("Connection closed.")

if __name__ == '__main__':
    main()