print("Hello!")
from machine import Pin, SPI
import utime
from bno08x import *
import mcp3008
import rp2
import omni
from MotorConfig import *
from hcsr04 import HCSR04

# CSピンをSPI0のペリフェラル領域外(例: Pin 15)に変更
cs_pin = Pin(1, Pin.OUT)

# baudrateを1MHz(1000000)に設定
spi = SPI(0, sck=Pin(2), mosi=Pin(3), miso=Pin(4), polarity=0, phase=0, baudrate=1000000)

# クラスに渡す
mcp = mcp3008.mcp(spi, cs_pin)

o = omni.Omni(PWM_LIST,IN_2_LIST,IN_1_LIST)
hcsr = HCSR04(0,Pin(14),Pin(15))

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
    print(f"\r[{'|'*f}{' '*(l-f)}] {int(n*100)}% ({v})", end=f"{"    !!LINE!!    " if v < shikii else ""}{c}")

shikii = 150
spd = 15000
senkai = 1
mae = 1.2
yoko = 0
check = False
waitime = 500
kst_rig = 0
kst_lft = 0
kstmv = 1 #交差点方向 -1左　1右
kstsnk = 1.3 #交差点旋回速度
ksty = 0.5 #交差点Vy値
kstime = 1000 #交差点曲がり時間
kstzzny = 1 #交差点事前前進Vy値
kszzntime = 700 #交差点事前前進時間
kst = 200 #交差点判定のはば
kst_tik = 200 #交差点後の待機
lost = False
lostime = 1000
losted = 0
latest = None
hcsr.activate()

sleep_ms(50)

try:
    while True:
        if rp2.bootsel_button() == 1:
            raise ValueError("bootsel")
        p(mcp.read_adc(0), 200, " s1\n")
        p(mcp.read_adc(1), 200, " s2\n")
        p(mcp.read_adc(2), 200, " s3\n")
        p(mcp.read_adc(3), 200, " s4\n\n\n\n")
        print(kst_lft,kst_rig)
        # if check :
        #     if wait == 0:
        #         if mcp.read_adc(0) < shikii or mcp.read_adc(3) < shikii:
        #             o.stop()
        #             check = False
        #             break
        #     else:
        #         wait -= 1
        #         spd -= 1
        kst_lft -= 1
        kst_rig -= 1
        kst_tik -= 1
        if mcp.read_adc(0)<shikii:
            kst_lft = kst
        if mcp.read_adc(3)<shikii:
            kst_rig = kst
        if kst_lft > 0 and kst_rig > 0 and kst_tik < 200 and kst_tik < 200:
            o.move(0,kstzzny,10000,0)
            sleep_ms(kszzntime)
            o.move(0.1,ksty*kstmv,10000,kstsnk*kstmv)
            sleep_ms(kstime)
            kst_rig,kst_lft=0,0
            kst_tik = 600
        if mcp.read_adc(1) < shikii and mcp.read_adc(2):
            check = True
            wait = waitime
            latestmove = "kst"
        if mcp.read_adc(1) < shikii or mcp.read_adc(0) < shikii:
            o.move(-yoko,mae,spd,-senkai)
            lost = False
            latestmove = "r"
        elif mcp.read_adc(2) < shikii:
            o.move(yoko,mae,spd,senkai)
            lost = False
            latestmove = "l"
        else:
            o.move(0,mae,spd,0)
            lost = True
            losted += 1
            if losted == lostime:
                o.stop()
                break
        if mcp.read_adc(3) < shikii:
            o.move(yoko,mae*0.5,spd,senkai*1.5)
            lost = False
            latestmove = "l"
        elif mcp.read_adc(0) < shikii:
            o.move(-yoko,mae*0.5,spd,-senkai*1.5)
            lost = False
            latestmove = "l"
        if lost:
            losted = 0
        if hcsr.distance() > 1 and hcsr.distance() < 10:
            o.move(0,-1,spd,0)
            latestmove = "b"
        # utime.sleep_ms(5) # ループが早すぎないように少しウェイトを入れる
except Exception as e:
    print("\nError:", e)
finally:
    o.stop()
    hcsr.deactivate()
    print("\nfinished?")
    led.off()