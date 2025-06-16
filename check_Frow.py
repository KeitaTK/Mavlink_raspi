from pymavlink import mavutil
import time
print("Pixhawk再起動後、921600bpsで接続テスト")
time.sleep(5)  # 再起動待ち

master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=921600, rtscts=True)
master.wait_heartbeat()
print("921600bps接続成功！")
