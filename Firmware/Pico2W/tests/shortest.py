from machine import Pin, PWM
import random
import time

pwm = PWM(Pin(0))
pwm.freq(3000)  # 周波数を1kHzに設定
pwm.duty_u16(2500)  # デューティ比を50%に設定

# from machine import Pin, ADC
# import time

# # GP26を内蔵プルアップ有効にした上でADCとして設定
# pin = Pin(26, Pin.IN, Pin.PULL_UP)
# adc = ADC(pin)

# # Pico 2 (RP2350) の内蔵プルアップ抵抗の目安値（約60kΩ）
# R_PULLUP = 60000 

# print("=== 簡易抵抗チェッカー開始 (GP26 - GND) ===")

# while True:
#     # 10回平均してノイズを減らす
#     raw = sum(adc.read_u16() for _ in range(10)) // 10
    
#     if raw < 150:
#         print(f"状態: 短絡 (0Ω付近 / 100Ω以下){raw}")
#     elif raw > 65000:
#         print("状態: 開放 (∞ Ω)")
#     else:
#         # 分圧の式から未知の抵抗値を逆算
#         r_unknown = R_PULLUP * raw / (65535 - raw)
#         if r_unknown >= 1000:
#             print(f"抵抗値: 約 {r_unknown / 1000:.1f} kΩ")
#         else:
#             print(f"抵抗値: 約 {r_unknown:.0f} Ω")
            
#     time.sleep(0.1)


# from machine import Pin
# import time

# # GP0を内蔵プルアップ有効で入力設定
# probe = Pin(0, Pin.IN, Pin.PULL_UP)

# # オンボードLED設定（目視でも確認できるように併用）
# led = Pin("LED", Pin.OUT)

# last_state = 1  # 前回の状態を記録（1=開放, 0=短絡）
# print("=== 簡易短絡テスター開始 (GP0 - GND) ===")

# while True:
#     current_state = probe.value()
    
#     # 状態が変化した時だけ表示とLEDを更新
#     if current_state != last_state:
#         if current_state == 0:
#             led.value(1)
#             print("【短絡！】SHORT DETECTED")
#         else:
#             led.value(0)
#             print("  開放   (OPEN)")
            
#         last_state = current_state
        
#     time.sleep_ms(10)  # チャタリング防止
