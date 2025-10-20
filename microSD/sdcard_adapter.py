# sdcard_adapter.py - spidev自動CS制御対応版
import spidev
import time

def const(x):
    return x

class Pin:
    """CS制御を spidev に委ねるダミーPinクラス"""
    OUT = 1
    
    def __init__(self, pin_num, mode=None):
        self.pin = pin_num
        # 実際のGPIO制御は行わない（spidevが自動制御）
        print(f"Pin {pin_num}: spidev自動制御モード")
    
    def init(self, mode=None):
        """MicroPython互換のinitメソッド（何もしない）"""
        pass
    
    def __call__(self, value):
        """CS制御はspidevが自動で行うため、何もしない"""
        pass
    
    def value(self, val=None):
        """値の設定/取得（ダミー）"""
        return 1 if val is None else None

class SPI:
    def __init__(self, bus=0, device=0):
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = 1000000  # 1MHz
        self.spi.mode = 0
        print(f"SPI初期化: /dev/spidev{bus}.{device}, 速度: {self.spi.max_speed_hz}Hz")
    
    def init(self, baudrate=1000000, polarity=0, phase=0):
        """MicroPython互換のinitメソッド"""
        self.spi.max_speed_hz = baudrate
        self.spi.mode = (polarity << 1) | phase
    
    def write(self, data):
        """データ書き込み"""
        if isinstance(data, int):
            # 単一の値の場合、CS制御を含めて送信
            self.spi.xfer2([data])
        elif isinstance(data, (bytes, bytearray)):
            # バイト列の場合、CS制御を含めて送信
            self.spi.xfer2(list(data))
        elif isinstance(data, list):
            # リストの場合、CS制御を含めて送信
            self.spi.xfer2(data)
    
    def read(self, length, write_data=0xFF):
        """データ読み込み"""
        read_data = [write_data] * length
        result = self.spi.xfer2(read_data)
        return bytes(result)
    
    def readinto(self, buf, write_data=0xFF):
        """バッファに読み込み"""
        read_data = [write_data] * len(buf)
        result = self.spi.xfer2(read_data)
        for i in range(min(len(result), len(buf))):
            buf[i] = result[i]
    
    def close(self):
        """SPI接続を閉じる"""
        self.spi.close()
