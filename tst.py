from machine import Pin
import time

# 今設定しているピン番号に書き換えて実行
trig = Pin(14, Pin.OUT)
echo = Pin(15, Pin.IN)

trig.value(0)
time.sleep_ms(100)

# 手動でパルスを出してEchoの変化を見る
trig.value(1)
time.sleep_us(10)
trig.value(0)

# Echoが立ち上がるかチェック
timeout = 10000
while echo.value() == 0 and timeout > 0:
    timeout -= 1

print("Echo High検知:", timeout > 0)