from machine import Pin
import time

# 今使っているピン番号に書き換えてください
TRIG_PIN = 14
ECHO_PIN = 15

trig = Pin(TRIG_PIN, Pin.OUT)
echo = Pin(ECHO_PIN, Pin.IN, Pin.PULL_DOWN)
print("プルダウン設定後のEcho生値:", echo.value())

trig.value(0)
time.sleep_ms(100)

def measure():
    # Trigパルス (10us)
    trig.value(1)
    time.sleep_us(10)
    trig.value(0)

    # Echoが1(High)になるのを待つ
    timeout = 100000
    while echo.value() == 0:
        timeout -= 1
        if timeout <= 0:
            return "Echo High待ちでタイムアウト (ずっとLow)"

    t_start = time.ticks_us()

    # Echoが0(Low)になるのを待つ
    timeout = 100000
    while echo.value() == 1:
        timeout -= 1
        if timeout <= 0:
            return "Echo Low待ちでタイムアウト (ずっとHigh)"

    t_end = time.ticks_us()

    # 経過時間(us)から距離計算 (cm)
    duration = time.ticks_diff(t_end, t_start)
    distance = (duration * 0.0343) / 2
    return f"{distance:.1f} cm (duration: {duration}us)"

# 5回測定
for i in range(5):
    print(f"[{i+1}]", measure())
    time.sleep_ms(200)