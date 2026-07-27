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
    print(f"\r[{'|'*f}{' '*(l-f)}] {int(n*100)}% ({v})", end=f"{"    !!LINE!!    " if v < shikii_line else ""}{c}")

o = omni.Omni(PWM_LIST,IN_2_LIST,IN_1_LIST,True)
shikii_dis = 10
dist = 0
shikii_line = 150
spd = 30000
senkai = 1
mae = 1.2
yoko = 0
check = False
waitime = 500
kst_rig = 0
kst_lft = 0
kstmv = -1 #交差点方向 -1左　1右
kstsnk = 1.3 #交差点旋回速度
ksty = 0.5 #交差点Vy値
kstime = 800 #交差点曲がり時間
kstzzny = 1 #交差点事前前進Vy値
kszzntime = 1200 #交差点事前前進時間
kst = 200 #交差点判定のはば
kst_tik = 200 #交差点後の待機
lost = False
lostime = 500
losted = 0
latest = None
hcsr.activate()

sleep_ms(50)

def normalTrace():
    global lost, latestmove, losted
    if mcp.read_adc(1) < shikii_line or mcp.read_adc(0) < shikii_line:
        o.move(-yoko,mae,spd,-senkai)
        lost = False
        latestmove = "r"
    elif mcp.read_adc(2) < shikii_line:
        o.move(yoko,mae,spd,senkai)
        lost = False
        latestmove = "l"
    else:
        o.move(0,mae,spd,0)
        if not lost:
            lost = True
            losted = utime.ticks_ms()

def kstCheck():
    global kst_lft,kst_rig,kst_tik
    kst_lft -= 1
    kst_rig -= 1
    kst_tik -= 1
    if mcp.read_adc(0)<shikii_line:
        kst_lft = kst
    if mcp.read_adc(3)<shikii_line:
        kst_rig = kst
    if kst_lft > 0 and kst_rig > 0 and kst_tik < 1:
        return True
    return False
    
def kstMove():
    global kst_rig,kst_lft,kst_tik,losted
    o.move(0,kstzzny,10000,0)
    sleep_ms(kszzntime)
    o.move(0.1,ksty*kstmv,10000,kstsnk*kstmv)
    sleep_ms(kstime)
    kst_rig,kst_lft,losted=0,0,utime.ticks_ms()
    kst_tik = 600
        
def sideTrace():
    global lost,latestmove
    if mcp.read_adc(3) < shikii_line:
        o.move(yoko,mae*0.5,spd,senkai*1.5)
        lost = False
        latestmove = "l"
    elif mcp.read_adc(0) < shikii_line:
        o.move(-yoko,mae*0.5,spd,-senkai*1.5)
        lost = False
        latestmove = "r"

def lostCheck():
    global losted, lost
    if lost and utime.ticks_diff(utime.ticks_ms(), losted) > lostime and kst_tik < 0:
        o.stop()
        for stp in range(50):
            o.move(0,0,5000,1)
            utime.sleep_ms(10)
            # if all(mcp.read_adc(i) > shikii_line for i in range(4)):
            if mcp.read_adc(1) < shikii_line or mcp.read_adc(2) < shikii_line:
                lost = False
                return False
        lost = False
        return True
    return False

def distCheck():
    global dist,dist_new
    dist_new = hcsr.distance()
    a = all([dist_new > (shikii_dis - 3), dist_new < shikii_dis, abs(dist_new - dist) < 5])
    dist = hcsr.distance()
    return a

try:
    while True:
        if rp2.bootsel_button() == 1:
            raise ValueError("bootsel")
        # p(mcp.read_adc(0), 200, " s1\n")
        # p(mcp.read_adc(1), 200, " s2\n")
        # p(mcp.read_adc(2), 200, " s3\n")
        # p(mcp.read_adc(3), 200, " s4\n\n\n\n")

        if kstCheck():
            print("INTERSECT DETECTED")
            kstMove()
        
        if lostCheck():
            print("LINE LOSTED")
            o.stop()
            break
        else:print("LINE DETECTED")

        if distCheck():
            print(f"TOO CLOSE {dist:.2f}cm")
            break
        else:print("FULLY")
        normalTrace()
        sideTrace()
        # utime.sleep_ms(5) # ループが早すぎないように少しウェイトを入れる
except Exception as e:
    print("\nError:", e)
finally:
    o.stop()
    hcsr.deactivate()
    print("\nfinished?")
    led.off()