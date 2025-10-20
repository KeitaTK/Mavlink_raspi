#!/usr/bin/env python3
# cs_control_test.py - CS制御付きSPIテスト

import spidev
import RPi.GPIO as GPIO
import time

def test_with_cs_control():
    """CS制御付きSPI通信テスト"""
    
    # GPIO初期化
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)
    
    CS_PIN = 8
    GPIO.setup(CS_PIN, GPIO.OUT)
    GPIO.output(CS_PIN, GPIO.HIGH)  # 初期状態はHIGH（非選択）
    
    try:
        # SPI初期化
        spi = spidev.SpiDev()
        spi.open(0, 0)  # /dev/spidev0.0 を使用
        spi.max_speed_hz = 400000  # 400kHz（初期化用の低速）
        spi.mode = 0
        
        print("=== CS制御付きSPIテスト ===")
        print(f"使用SPI: /dev/spidev0.0")
        print(f"速度: {spi.max_speed_hz}Hz")
        print(f"CS Pin: GPIO{CS_PIN}")
        
        # 1. 初期化シーケンス
        print("\n1. 初期化シーケンス開始")
        GPIO.output(CS_PIN, GPIO.HIGH)
        time.sleep(0.1)
        
        # ダミークロック送信（SDカード起動用）
        print("2. ダミークロック送信（SDカード起動）")
        dummy_clocks = [0xFF] * 10
        spi.xfer2(dummy_clocks)
        time.sleep(0.01)
        
        # 3. CMD0 (GO_IDLE_STATE) 送信
        print("3. CMD0送信開始")
        GPIO.output(CS_PIN, GPIO.LOW)  # CS LOW（選択）
        time.sleep(0.001)  # CS setup time
        
        cmd0 = [0x40, 0x00, 0x00, 0x00, 0x00, 0x95]  # CMD0 + 正しいCRC
        print(f"   送信データ: {[hex(x) for x in cmd0]}")
        
        # CMD0送信
        response = spi.xfer2(cmd0)
        print(f"   即座の応答: {[hex(x) for x in response]}")
        
        # 4. R1応答待ち（最大8バイト読み込み）
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
                elif resp_byte & 0x80:
                    print(f"   ⚠ エラー状態: {bin(resp_byte)}")
                else:
                    print(f"   △ 未知の応答: {hex(resp_byte)}")
                    valid_response = True
                break
        
        GPIO.output(CS_PIN, GPIO.HIGH)  # CS HIGH（非選択）
        spi.write([0xFF])  # クリーンアップクロック
        
        # 5. 結果判定
        print(f"\n=== テスト結果 ===")
        if valid_response:
            print("✓ SDカードとの通信が確認できました")
            print("  → 配線とSDカードは正常です")
            print("  → 次のステップ: SDカードライブラリのテスト")
        else:
            print("✗ SDカードからの応答がありません")
            print("  → 確認項目:")
            print("    - microSDカードが正しく挿入されているか")
            print("    - 配線が正しいか（特にMISO線）")
            print("    - SDカードが動作可能な状態か")
        
        spi.close()
        
    except Exception as e:
        print(f"エラー: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    test_with_cs_control()
