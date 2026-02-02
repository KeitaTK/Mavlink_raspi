from pymavlink import mavutil
import time

def get_param_value(master, param_name, timeout=5):
    """
    指定したパラメータ名の値を読み出して返す。
    """
    master.mav.param_request_read_send(
        master.target_system,
        master.target_component,
        param_name.encode('ascii'),
        -1
    )
    start = time.time()
    while time.time() - start < timeout:
        msg = master.recv_match(type='PARAM_VALUE', blocking=True, timeout=timeout)
        if not msg:
            continue
        # param_id が bytes か str かをチェックして取り出す
        pid = msg.param_id
        if isinstance(pid, (bytes, bytearray)):
            pid = pid.decode('ascii').strip('\x00')
        else:
            pid = pid.strip('\x00')
        if pid == param_name:
            return msg.param_value
    raise TimeoutError(f"パラメータ {param_name} の応答がありませんでした")

if __name__ == '__main__':
    try:
        print("接続中: /dev/ttyAMA0 @ 1000000bps, RTS/CTS enabled")
        master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=1000000, rtscts=True)
        master.wait_heartbeat(timeout=5)
        print(f"Heartbeat 受信: system={master.target_system}, component={master.target_component}")

        param_name = "OBS_CORR_GAIN"
        print(f"{param_name} の値を取得中...")
        value = get_param_value(master, param_name)
        print(f"{param_name} = {value}")

    except Exception as e:
        print(f"エラー: {e}")

    finally:
        try:
            master.close()
        except:
            pass
