#!/usr/bin/env python3
# simple_spi_test.py - 最も簡単なテスト

import spidev
import time

def simple_test():
    """最も簡単なSPI接続テスト"""
    try:
        spi = spidev.SpiDev()
        spi.open(0, 0)
        spi.max_speed_hz = 100000  # 100kHz
        
        print("SPI接続テスト開始...")
        
        # ダミーデータを送信
        test_data = [0xFF, 0xFF, 0xFF]
        response = spi.xfer2(test_data)
        
        print(f"送信: {test_data}")
        print(f"受信: {response}")
        
        if response != [0xFF, 0xFF, 0xFF]:
            print("✓ 何らかの応答があります")
        else:
            print("△ 応答が全て0xFF（接続未確認）")
        
        spi.close()
        
    except Exception as e:
        print(f"エラー: {e}")

if __name__ == "__main__":
    simple_test()
