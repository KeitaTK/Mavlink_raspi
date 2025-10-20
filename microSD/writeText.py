#!/usr/bin/env python3
# 取得したsdcard.pyを使用

import spidev
import time
from sdcard import SDCard  # 取得したライブラリをインポート

class GPIO_Pin:
    """GPIO制御クラス（Raspberry Pi用）"""
    def __init__(self, pin_num):
        import RPi.GPIO as GPIO
        GPIO.setmode(GPIO.BCM)
        self.pin = pin_num
        GPIO.setup(self.pin, GPIO.OUT)
    
    def __call__(self, value):
        import RPi.GPIO as GPIO
        GPIO.output(self.pin, value)

class SPI_Wrapper:
    """spidevをMicroPython風にラップ"""
    def __init__(self, bus=0, device=0):
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = 1000000
        self.spi.mode = 0
    
    def write(self, data):
        if isinstance(data, (bytes, bytearray)):
            self.spi.writebytes(list(data))
        else:
            self.spi.writebytes([data])
    
    def read(self, length, dummy_byte=0xFF):
        return self.spi.readbytes(length)

# 使用例
def main():
    try:
        # SPI初期化
        spi = SPI_Wrapper(0, 0)  # SPI0, CS0
        cs_pin = GPIO_Pin(8)     # GPIO 8をCS用
        
        # SDカード初期化
        sd = SDCard(spi, cs_pin)
        
        print("SDカードの初期化が完了しました")
        print(f"セクタ数: {sd.sectors}")
        
        # テストデータの書き込み
        test_data = b"GitHub経由で取得したsdcard.pyのテスト\n" * 10
        block_data = bytearray(512)  # 512バイトブロック
        block_data[:len(test_data)] = test_data
        
        # ブロック0に書き込み
        sd.writeblocks(0, block_data)
        print("データの書き込み完了")
        
        # 書き込んだデータを読み込み
        read_data = bytearray(512)
        sd.readblocks(0, read_data)
        
        print("読み込みデータ:")
        print(read_data[:len(test_data)].decode('utf-8'))
        
    except Exception as e:
        print(f"エラー: {e}")

if __name__ == "__main__":
    main()
