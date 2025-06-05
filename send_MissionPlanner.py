from pymavlink import mavutil
import time

# 接続設定
PORT = '/dev/ttyAMA0'   # TELEM1 UART
BAUD = 115200
UDP_HOST = '192.168.11.46'   # Mission PlannerのIPアドレス
UDP_PORT = 14550        # Mission Plannerが待機するUDPポート

def fix_mavlink_message_for_forward(msg):
    """
    MAVLinkメッセージを転送用に修正
    文字列を含むメッセージのエンコーディング問題を解決
    """
    msg_type = msg.get_type()
    if msg_type in ('PARAM_VALUE', 'PARAM_REQUEST_READ', 'PARAM_SET'):
        if type(msg.param_id) == str:
            msg.param_id = msg.param_id.encode()
    elif msg_type == 'STATUSTEXT':
        if type(msg.text) == str:
            msg.text = msg.text.encode()
    return msg

def main():
    try:
        # ArduPilotとのシリアル接続を確立
        print(f"ArduPilotに接続中... ({PORT}, {BAUD} baud)")
        ardupilot_conn = mavutil.mavlink_connection(f'{PORT}:{BAUD}')
        ardupilot_conn.wait_heartbeat()
        print(f"ArduPilotからハートビート受信 (system {ardupilot_conn.target_system} component {ardupilot_conn.target_component})")
        
        # Mission PlannerへのUDP接続を確立（出力用）
        print(f"Mission Planner UDP接続を準備中... ({UDP_HOST}:{UDP_PORT})")
        udp_conn = mavutil.mavlink_connection(f'udpout:{UDP_HOST}:{UDP_PORT}')
        print("UDP接続準備完了")
        
        print("メッセージ転送を開始...")
        message_count = 0
        
        while True:
            # ArduPilotからメッセージを受信（ノンブロッキング）
            msg = ardupilot_conn.recv_match(blocking=False)
            
            if msg is not None and msg.get_type() != 'BAD_DATA':
                # メッセージを転送用に修正
                msg = fix_mavlink_message_for_forward(msg)
                
                # 元のソースシステム/コンポーネントIDを保持して転送
                udp_conn.mav.srcSystem = msg.get_srcSystem()
                udp_conn.mav.srcComponent = msg.get_srcComponent()
                
                # Mission PlannerにUDP経由で送信
                udp_conn.mav.send(msg)
                
                message_count += 1
                if message_count % 100 == 0:  # 100メッセージごとに進捗を表示
                    print(f"転送メッセージ数: {message_count}")
            
            # CPU使用率を抑制
            time.sleep(0.001)
            
    except KeyboardInterrupt:
        print("\n転送を停止中...")
    except Exception as e:
        print(f"エラーが発生しました: {e}")
    finally:
        print("接続を終了しています...")
        if 'ardupilot_conn' in locals():
            ardupilot_conn.close()
        if 'udp_conn' in locals():
            udp_conn.close()
        print("プログラムを終了しました")

if __name__ == "__main__":
    main()
