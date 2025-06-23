#!/usr/bin/env python3
"""
Raspberry Pi 5 シリアル通信テスト
"""

import serial
import time

def test_pi5_serial_devices():
    """Pi5の複数シリアルデバイステスト"""
    
    devices_to_test = [
        '/dev/serial0',    # -> ttyAMA10
        '/dev/ttyAMA10',   # 直接指定
        '/dev/ttyAMA0',    # 代替デバイス
    ]
    
    baudrates = [115200, 57600]
    
    for device in devices_to_test:
        for baud in baudrates:
            print(f"\n=== Testing {device} at {baud} baud ===")
            
            try:
                ser = serial.Serial(device, baud, timeout=0.5)
                print(f"✅ Opened {device}")
                
                # データ受信テスト
                print("Listening for 3 seconds...")
                start_time = time.time()
                data_received = 0
                mavlink_patterns = 0
                
                while time.time() - start_time < 3:
                    if ser.in_waiting > 0:
                        data = ser.read(ser.in_waiting)
                        data_received += len(data)
                        
                        # MAVLinkパターン検索
                        for byte in data:
                            if byte in [0xFE, 0xFD]:  # MAVLink magic bytes
                                mavlink_patterns += 1
                                print(f"   MAVLink pattern found: 0x{byte:02X}")
                    
                    time.sleep(0.1)
                
                print(f"   Total bytes: {data_received}")
                print(f"   MAVLink patterns: {mavlink_patterns}")
                
                if data_received > 0:
                    print(f"✅ {device} at {baud}: Data received!")
                    ser.close()
                    return device, baud
                else:
                    print(f"❌ {device} at {baud}: No data")
                
                ser.close()
                
            except Exception as e:
                print(f"❌ {device} at {baud}: {e}")
    
    return None, None

if __name__ == "__main__":
    working_device, working_baud = test_pi5_serial_devices()
    
    if working_device:
        print(f"\n🎉 Working configuration found:")
        print(f"   Device: {working_device}")
        print(f"   Baud: {working_baud}")
    else:
        print(f"\n❌ No working configuration found")
        print("   Check physical wiring and Pixhawk power")


