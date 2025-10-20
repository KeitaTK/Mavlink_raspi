# sdcard_adapter.py - Raspberry Pi 5用アダプター
import spidev
import RPi.GPIO as GPIO
import time

def const(x):
    return x

class Pin:
    OUT = GPIO.OUT
    
    def __init__(self, pin_num, mode=None):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        self.pin = pin_num
        if mode:
            GPIO.setup(self.pin, mode)
    
    def __call__(self, value):
        GPIO.output(self.pin, value)

class SPI:
    def __init__(self, bus=0, device=0):
        self.spi = spidev.SpiDev()
        self.spi.open(bus, device)
        self.spi.max_speed_hz = 1000000
        self.spi.mode = 0
    
    def write(self, data):
        if isinstance(data, int):
            data = [data]
        elif isinstance(data, (bytes, bytearray)):
            data = list(data)
        self.spi.writebytes(data)
    
    def read(self, length, write_data=0xFF):
        return bytes(self.spi.readbytes(length))
    
    def readinto(self, buf, write_data=0xFF):
        data = self.spi.readbytes(len(buf))
        for i in range(min(len(data), len(buf))):
            buf[i] = data[i]
    
    def close(self):
        self.spi.close()
