from pymavlink import mavutil
import time

def get_param(master, name, timeout=5):
    # パラメータ取得リクエスト
    master.mav.param_request_read_send(
        master.target_system,
        master.target_component,
        name.encode('utf-8'),
        -1
    )
    # PARAM_VALUE を待機
    start = time.time()
    while time.time() - start < timeout:
        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=1)
        if msg:
            # Python 3対応: param_idが既に文字列の場合とbytesの場合の両方に対応
            if isinstance(msg.param_id, bytes):
                param_id = msg.param_id.decode('utf-8').strip('\x00')
            else:
                param_id = str(msg.param_id).strip('\x00')
            
            if param_id == name:
                return msg.param_value
    return None

if __name__ == '__main__':
    # シリアル接続（TELEM1, 1 000 000bps, RTS/CTS 有効）
    master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=1000000, rtscts=True)
    try:
        # Heartbeat を待ってからリクエスト
        master.wait_heartbeat(timeout=5)
        # 一度だけ MOT_THST_HOVER を取得して表示
        hover = get_param(master, 'MOT_THST_HOVER')
        if hover is not None:
            print(f'MOT_THST_HOVER = {hover:.3f}')
        else:
            print('MOT_THST_HOVER の取得に失敗しました')
    finally:
        master.close()

