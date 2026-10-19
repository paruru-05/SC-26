import machine
import ssd1306
import time
import micropython

@micropython.viper
def _get_diff_bounds(old_buf: ptr8, new_buf: ptr8, width: int, pages: int) -> object:
    min_c = 999
    max_c = -1
    min_p = 999
    max_p = -1
    
    for p in range(pages):
        for c in range(width):
            idx = p * width + c
            val = new_buf[idx]
            
            if old_buf[idx] != val:
                old_buf[idx] = val
                if c < min_c: min_c = c
                if c > max_c: max_c = c
                if p < min_p: min_p = p
                if p > max_p: max_p = p
                
    return (min_c, max_c, min_p, max_p)

def show_auto_diff(oled, old_buffer):
    min_c, max_c, min_p, max_p = _get_diff_bounds(old_buffer, oled.buffer, oled.width, oled.pages)
    
    if max_c == -1:
        return

    oled.write_cmd(0x21) # SET_COL_ADDR
    oled.write_cmd(min_c)
    oled.write_cmd(max_c)
    
    oled.write_cmd(0x22) # SET_PAGE_ADDR
    oled.write_cmd(min_p)
    oled.write_cmd(max_p)

    for p in range(min_p, max_p + 1):
        start = p * oled.width + min_c
        end = start + (max_c - min_c + 1)
        oled.write_data(oled.buffer[start:end])

def run_benchmark(oled):
    old_buffer = bytearray(1024)
    iterations = 50
    
    print("=" * 40)
    print(" SSD1306 Auto-Diff Benchmark (I2C)")
    print("=" * 40)
    
    # ----------------------------------------------------
    # テスト1: 小領域の更新 (カウンターのカウントアップ)
    # ----------------------------------------------------
    print("\n[Test 1] Small Area Update (Counter)")
    
    # Normal show() 計測
    normal_times = []
    oled.fill(0)
    for i in range(iterations):
        oled.fill_rect(10, 10, 40, 16, 0)
        oled.text(str(i), 10, 10)
        start = time.ticks_us()
        oled.show()
        normal_times.append(time.ticks_diff(time.ticks_us(), start))
        
    # Auto-Diff 計測
    diff_times = []
    oled.fill(0)
    for i in range(1024): old_buffer[i] = 0 # 比較バッファをリセット
    for i in range(iterations):
        oled.fill_rect(10, 10, 40, 16, 0)
        oled.text(str(i), 10, 10)
        start = time.ticks_us()
        show_auto_diff(oled, old_buffer)
        diff_times.append(time.ticks_diff(time.ticks_us(), start))
        
    print(f"Normal show() : {sum(normal_times) / len(normal_times) / 1000:.2f} ms")
    print(f"Auto-Diff     : {sum(diff_times) / len(diff_times) / 1000:.2f} ms")

    # ----------------------------------------------------
    # テスト2: 全画面の更新 (毎回全画面を白黒反転)
    # ----------------------------------------------------
    print("\n[Test 2] Full Screen Update (Worst Case)")
    
    normal_times = []
    for i in range(iterations):
        oled.fill(i % 2) # 0(黒)と1(白)を交互に全画面塗りつぶし
        start = time.ticks_us()
        oled.show()
        normal_times.append(time.ticks_diff(time.ticks_us(), start))
        
    diff_times = []
    for i in range(1024): old_buffer[i] = oled.buffer[i]
    for i in range(iterations):
        oled.fill(i % 2)
        start = time.ticks_us()
        show_auto_diff(oled, old_buffer)
        diff_times.append(time.ticks_diff(time.ticks_us(), start))
        
    print(f"Normal show() : {sum(normal_times) / len(normal_times) / 1000:.2f} ms")
    print(f"Auto-Diff     : {sum(diff_times) / len(diff_times) / 1000:.2f} ms")

    # ----------------------------------------------------
    # テスト3: 変化なし (画面を描き換えない)
    # ----------------------------------------------------
    print("\n[Test 3] No Change (Best Case)")
    
    normal_times = []
    for i in range(iterations):
        # 描画処理なし
        start = time.ticks_us()
        oled.show()
        normal_times.append(time.ticks_diff(time.ticks_us(), start))
        
    diff_times = []
    for i in range(1024): old_buffer[i] = oled.buffer[i]
    for i in range(iterations):
        # 描画処理なし
        start = time.ticks_us()
        show_auto_diff(oled, old_buffer)
        diff_times.append(time.ticks_diff(time.ticks_us(), start))
        
    print(f"Normal show() : {sum(normal_times) / len(normal_times) / 1000:.2f} ms")
    print(f"Auto-Diff     : {sum(diff_times) / len(diff_times) / 1000:.2f} ms")
    print("=" * 40)

if __name__ == "__main__":
    # ※ お使いのマイコンに合わせてピン番号を変更してください
    # (例: ESP32なら scl=Pin(22), sda=Pin(21))
    # (例: Pi Picoなら scl=Pin(5), sda=Pin(4), id=0)
    i2c = machine.I2C(1,sda=machine.Pin(26),scl=machine.Pin(27),freq=3000000)
    
    # SSD1306の初期化 (128x64サイズ)
    oled = ssd1306.SSD1306_I2C(128, 64, i2c)
    
    # ベンチマーク実行！
    run_benchmark(oled)