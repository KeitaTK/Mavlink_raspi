# より高速な通信速度を試してみる
baud_test = [921600, 1000000, 1500000]

for baud in baud_test:
    try:
        print(f"Testing {baud}bps...")
        master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=baud, rtscts=True)
        master.wait_heartbeat(timeout=3)
        print(f"{baud}bps: 成功！")
        
        # 簡単な通信テスト
        master.mav.autopilot_version_request_send(master.target_system, master.target_component)
        msg = master.recv_match(type='AUTOPILOT_VERSION', blocking=True, timeout=2)
        if msg:
            print(f"  → データ受信も成功")
        master.close()
    except Exception as e:
        print(f"{baud}bps: 失敗 - {e}")
