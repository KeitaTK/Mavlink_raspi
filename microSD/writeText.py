#!/usr/bin/env python3
# pmod_sdcard_simple.py - Pmod MicroSD簡単書き込み

import os
import time
from datetime import datetime
from sdcard_adapter import Pin, SPI, const
from sdcard import SDCard

class PmodSDCard:
    def __init__(self):
        """Pmod MicroSDを初期化"""
        try:
            # SPI初期化（SPI0, CE0）
            self.spi = SPI(0, 0)
            
            # CS pin 初期化（GPIO 8）
            self.cs = Pin(8, Pin.OUT)
            
            # SDカード初期化
            print("Pmod MicroSDを初期化中...")
            self.sd = SDCard(self.spi, self.cs)
            print(f"初期化完了（セクタ数: {self.sd.sectors}）")
            
        except Exception as e:
            print(f"初期化エラー: {e}")
            raise
    
    def write_text_block(self, block_num, text):
        """指定ブロックにテキストを書き込み"""
        try:
            # テキストを512バイトブロックに変換
            data = text.encode('utf-8')
            buffer = bytearray(512)
            
            if len(data) > 512:
                data = data[:512]  # 512バイト以内に切り詰め
            
            buffer[:len(data)] = data
            
            # SDカードに書き込み
            self.sd.writeblocks(block_num, buffer)
            print(f"ブロック {block_num} に書き込み完了")
            return True
            
        except Exception as e:
            print(f"書き込みエラー: {e}")
            return False
    
    def read_text_block(self, block_num):
        """指定ブロックからテキストを読み込み"""
        try:
            buffer = bytearray(512)
            self.sd.readblocks(block_num, buffer)
            
            # null文字までを取得してデコード
            end_pos = buffer.find(0)
            if end_pos == -1:
                end_pos = 512
            
            text = buffer[:end_pos].decode('utf-8', errors='ignore')
            print(f"ブロック {block_num} から読み込み完了")
            return text
            
        except Exception as e:
            print(f"読み込みエラー: {e}")
            return None
    
    def append_log(self, message):
        """ログメッセージを追記（ブロック1を使用）"""
        try:
            # 既存データを読み込み
            existing = self.read_text_block(1) or ""
            
            # タイムスタンプ付きメッセージを追加
            timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            new_entry = f"[{timestamp}] {message}\n"
            
            # 新しい内容を作成
            updated_content = existing + new_entry
            
            # 512バイト以内に収まるように調整
            if len(updated_content.encode('utf-8')) > 500:
                lines = updated_content.split('\n')
                # 古いエントリから削除
                while len('\n'.join(lines).encode('utf-8')) > 500 and len(lines) > 1:
                    lines.pop(0)
                updated_content = '\n'.join(lines)
            
            # 書き込み
            return self.write_text_block(1, updated_content)
            
        except Exception as e:
            print(f"ログ追記エラー: {e}")
            return False
    
    def close(self):
        """リソースを解放"""
        try:
            self.spi.close()
            import RPi.GPIO as GPIO
            GPIO.cleanup()
            print("リソース解放完了")
        except:
            pass

def main():
    """使用例"""
    print("=== Pmod MicroSD 簡単書き込みテスト ===")
    
    try:
        # Pmod SDカード初期化
        pmod_sd = PmodSDCard()
        
        # テストメッセージ
        test_message = f"""
Pmod MicroSD テスト

実行日時: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}
Raspberry Pi 5 + Pmod MicroSD
SPI通信によるSDカードアクセス

このメッセージはブロック0に保存されています。
        """.strip()
        
        print("\n1. テキスト書き込みテスト")
        if pmod_sd.write_text_block(0, test_message):
            print("✓ 書き込み成功")
        
        print("\n2. テキスト読み込みテスト")
        read_data = pmod_sd.read_text_block(0)
        if read_data:
            print("✓ 読み込み成功")
            print(f"読み込み内容:\n{read_data}")
        
        print("\n3. ログ機能テスト")
        pmod_sd.append_log("システム開始")
        pmod_sd.append_log("テスト実行中")
        pmod_sd.append_log("処理完了")
        
        # ログ内容確認
        log_content = pmod_sd.read_text_block(1)
        if log_content:
            print("ログ内容:")
            print(log_content)
        
        print("\n✓ 全テスト完了")
        
    except KeyboardInterrupt:
        print("\n中断されました")
    except Exception as e:
        print(f"\nエラー: {e}")
    finally:
        try:
            pmod_sd.close()
        except:
            pass

if __name__ == "__main__":
    main()
