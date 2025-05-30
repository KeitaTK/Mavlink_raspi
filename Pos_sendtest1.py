from pymavlink import mavutil
import time

# 接続確立
master = mavutil.mavlink_connection('/dev/ttyACM0', baud=57600)

# カスタムメッセージが利用可能か確認
if hasattr(master.mav, 'taki_pos_motive_send'):
    print("Custom message available!")
    master.mav.taki_pos_motive_send(
        1.5,  # x_coord
        2.3,  # y_coord
        3.7   # z_coord
    )
    print("TAKI_POS_MOTIVE sent: x=1.5, y=2.3, z=3.7")
else:
    print("Custom message NOT available!")
    # 利用可能なメソッドを確認
    methods = [method for method in dir(master.mav) if 'send' in method.lower()]
    print("Available send methods:", methods[:10])
