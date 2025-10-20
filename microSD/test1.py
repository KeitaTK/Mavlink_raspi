#!/usr/bin/env python3
# cs_test_lgpio.py - lgpio使用版

import spidev
import lgpio
import time

def test_with_lgpio():
    """lgpioを使用したCS制御付きSPIテスト"""
    
    # GPIO初期化
    try:
        h = lgpio.gpiochip_open(0)  # GPIO chip 0を開く
    except Exception as e:
        print(f"GPIOチップオープンエラー: {e}")
        return
    
    CS_PIN = 8
    
    try:
        # CS pinを出力として設定
        lgpio.gpio_claim_output(h, CS_PIN)
        lgpio.gpio_write(h, CS_PIN, 1)  # 初期状態はHIGH（非選択）
        
        # SPI初期化
        spi = spidev.SpiDev()
        spi.open(0, 0)  # /dev/spidev0.0 を使用
        spi.max_speed_hz = 400000  # 400kHz
        spi.mode = 0
        
        print("=== lgpio使用 CS制御付きSPIテスト ===")
        print(f"使用SPI: /dev/spidev0.0")
        print(f"速度: {spi.max_speed_hz}Hz")
        print(f"CS Pin: GPIO{CS_PIN}")
        
        # 1. 初期化シーケンス
        print("\n1. 初期化シーケンス開始")
        lgpio.gpio_write(h, CS_PIN, 1)  # CS HIGH
        time.sleep(0.1)
        
        # ダミークロック送信
        print("2. ダミークロック送信")
        dummy_clocks = [0xFF] * 10
        spi.xfer2(dummy_clocks)
        time.sleep(0.01)
        
        # 3. CMD0送信
        print("3. CMD0送信開始")
        lgpio.gpio_write(h, CS_PIN, 0)  # CS LOW（選択）
        time.sleep(0.001)
        
        cmd0 = [0x40, 0x00, 0x00, 0x00, 0x00, 0x95]
        print(f"   送信データ: {[hex(x) for x in cmd0]}")
        
        response = spi.xfer2(cmd0)
        print(f"   即座の応答: {[hex(x) for x in response]}")
        
        # 4. R1応答待ち
        print("4. R1応答待ち")
        valid_response = False
        
        for i in range(8):
            resp_byte = spi.xfer2([0xFF])[0]
            print(f"   応答バイト{i+1}: {hex(resp_byte)}")
            
            if resp_byte != 0xFF:
                print(f"   ✓ 有効な応答を受信: {hex(resp_byte)}")
                
                if resp_byte == 0x01:
                    print("   ✓ SDカードがIDLE状態（正常）")
                    valid_response = True
                elif resp_byte == 0x00:
                    print("   ✓ SDカードが正常状態")
                    valid_response = True
                else:
                    print(f"   △ 応答: {hex(resp_byte)}")
                    valid_response = True
                break
        
        lgpio.gpio_write(h, CS_PIN, 1)  # CS HIGH（非選択）
        spi.write([0xFF])
        
        # 5. 結果判定
        print(f"\n=== テスト結果 ===")
        if valid_response:
            print("✓ SDカードとの通信が確認できました")
        else:
            print("✗ SDカードからの応答がありません")
        
        spi.close()
        
    except Exception as e:
        print(f"エラー: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        lgpio.gpiochip_close(h)

if __name__ == "__main__":
    test_with_lgpio()
