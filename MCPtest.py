print("Hello!")
from machine import Pin, SPI
import utime
from bno08x import *
import mcp3008
import rp2

# CSピンをSPI0のペリフェラル領域外(例: Pin 15)に変更
cs_pin = Pin(15, Pin.OUT)

# baudrateを1MHz(1000000)に設定
spi = SPI(0, sck=Pin(2), mosi=Pin(3), miso=Pin(4), polarity=0, phase=0, baudrate=1000000)

# クラスに渡す
mcp = mcp3008.mcp(spi, cs_pin)

led = Pin("LED", Pin.OUT)
led.on()

def is_bootsel_pressed():
    return rp2.bootsel_button() == 1

led.off()

print("start")
led.on()
utime.sleep(1)

def p(v, l=20, c=""):
    n = max(0, min(1, v / 516))  # 10bit ADCなので最大値は1023
    f = int(n * l)
    print(f"\r[{'|'*f}{' '*(l-f)}] {int(n*100)}% ({v})", end=f"{"    !!LINE!!    " if v < 100 else ""}{c}")

try:
    while True:
        if rp2.bootsel_button() == 1:
            raise ValueError("bootsel")
        #p(mcp.read_adc(0), 200, " s1\n")
        p(mcp.read_adc(1), 200, " s2\n")
        p(mcp.read_adc(2), 200, " s3\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n")
        #p(mcp.read_adc(3), 200, " s4\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n\n")
        utime.sleep_ms(50)  # ループが早すぎないように少しウェイトを入れる
except Exception as e:
    print("\nError:", e)
finally:
    print("\nfinished?")
    led.off()