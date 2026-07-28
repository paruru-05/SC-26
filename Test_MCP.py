print("Hello!")
from machine import Pin, SPI
import utime
import mcp3008
import rp2
import omni
from MotorConfig import *
from hcsr04 import HCSR04

# CSピンをSPI0のペリフェラル領域外(例: Pin 15)に変更
# baudrateを1MHz(1000000)に設定
spi = SPI(0, sck=Pin(2), mosi=Pin(3), miso=Pin(4), polarity=0, phase=0, baudrate=1000000)
cs_pin = Pin(1, Pin.OUT)

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

def normalTrace():
    global lost, latestmove, losted
    if mcp.read_adc(1) < shikii_line:
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

def kstCheck_T():
    global kst_lft,kst_rig,kst_tik,skwww
    kst_lft -= 1
    kst_rig -= 1
    kst_tik -= 1
    # d1 = [mcp.read_adc(1) > shikii_line, mcp.read_adc(2) > shikii_line, mcp.read_adc(3) > shikii_line]
    # d2 = [mcp.read_adc(0) > shikii_line, mcp.read_adc(1) > shikii_line, mcp.read_adc(2) > shikii_line]
    d1,d2 = [],[]
    if mcp.read_adc(0)<shikii_line and all(d1):
        kst_lft = kst
    if mcp.read_adc(3)<shikii_line and all(d2):
        kst_rig = kst
    if kst_lft > 0 and kst_rig > 0 and kst_tik < 1:
        skwww = "T"
        return True
    return False

def kstMove():
    global kst_rig,kst_lft,kst_tik,losted,skwww
    o.move(0,kstzzny,10000,0)
    utime.sleep_ms(kszzntime)
    o.move(0.1,ksty*kstmv,10000,kstsnk*kstmv)
    utime.sleep_ms(kstime)
    kst_rig,kst_lft,losted=0,0,utime.ticks_ms()
    kst_tik = 600
    skwww = ""
        
    
def kstCheck_skw():
    global kst_lft,kst_rig,kst_tik
    if not lost:
        if mcp.read_adc(0)<(shikii_line+50):
            return "lft"
        if mcp.read_adc(3)<(shikii_line+50):
            return "rig"
        print("NO SKW")
    return False

def kstMove_skw(dir:str):
    global kst_rig,kst_lft,kst_tik,losted,skwww,skwtime
    o.move(0,kstzzny,10000,0)
    utime.sleep_ms(kszzntime)
    if dir == "lft":
        o.move(0.1,ksty*kstmv,10000,kstsnk*-1)
    else:
        o.move(0.1,ksty*kstmv,10000,kstsnk*1)
    utime.sleep_ms(kstime)
    kst_rig,kst_lft,losted=0,0,utime.ticks_ms()
    kst_tik = 600
    skwtime = 0
    skwww = ""
    
def sideTrace():
    global lost,latestmove,side,losted,skw_waitime
    side = True
    if lost and skwtime == 0:
        if mcp.read_adc(3) < shikii_line:
            o.move(0.5,0,spd,senkai*3)
            latestmove = "l"
            losted = utime.ticks_ms()
            side = True
        elif mcp.read_adc(0) < shikii_line:
            o.move(-0.5,-0.1,spd,-senkai*3)
            latestmove = "r"
            losted = utime.ticks_ms()
            side = True
        if mcp.read_adc(1) < shikii_line or mcp.read_adc(2) < shikii_line:
            lost = False

def lostCheck():
    global losted, lost
    if lost and utime.ticks_diff(utime.ticks_ms(), losted) > lostime and kst_tik < 0:
        o.stop()
        for stp in range(100):
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

o = omni.Omni(PWM_LIST,IN_2_LIST,IN_1_LIST,True)
shikii_dis = 10
dist = 0
shikii_line = 150
spd = 15000
senkai = 1
mae = 0.8
yoko = 0
check = False
waitime = 500
kst_rig = 0
kst_lft = 0
kstmv = -1 #交差点方向 -1左　1右
kstsnk = 1.3 #交差点旋回速度
ksty = 0.5 #交差点Vy値
kstime = 750 #交差点曲がり時間
kstzzny = 1 #交差点事前前進Vy値
kszzntime = 700 #交差点事前前進時間
kst = 200 #交差点判定のはば
kst_tik = 50 #交差点後の待機
skw = 1 #ト字の検知のはば
skwwait = 100 #ト字検知後のT字待機
skw_waitime = 0
skwtime = 0
skwww = ""
lost = False
lostime = 500
losted = 0
latest = None
latestmove = None
hcsr.activate()

try:
    while True:
        if rp2.bootsel_button() == 1:
            raise ValueError("bootsel")
        # p(mcp.read_adc(0), 200, " s1\n")
        # p(mcp.read_adc(1), 200, " s2\n")
        # p(mcp.read_adc(2), 200, " s3\n")
        # p(mcp.read_adc(3), 200, " s4\n\n\n\n")

        
        
        if lostCheck():
            print("LINE LOSTED")
            o.stop()
            break

        if distCheck():
            o.stop()
            utime.sleep_ms(100)
            print(f"too close? {dist:.2f}cm")
            if distCheck():
                print(f"TOO CLOSE {dist:.2f}cm")
                break
        else:pass
        
        if kstCheck_T():
            print("T INTERSECT",latestmove)
            kstMove()
        
        normalTrace()        


        if skw_waitime == 0:
            skwww = kstCheck_skw()
            if skwww:
                if skwww == "lft":
                    print("LEFT INTERSECT?")
                    skwtime -= 1
                else:
                    print("RIGHT INTERSECT?")
                    skwtime += 1
            else:skwww = ""

        
            if skwtime >= skw:
                print("RIGHT")
                skw_waitime = skwwait
                kstMove_skw("rig")
            elif skwtime <= -skw:
                print("LEFT")
                skw_waitime = skwwait
                kstMove_skw("lft")
            # else:
            #     # 反応がないときはしっかりクリア
            #     skwtime = 0
            #     skwww = ""
        elif skw_waitime == 1:
            if not skwww == "T" and skw:
                print(f"SKW {skwww}")
            skw_waitime = 0
        elif skw_waitime > 0:
            skw_waitime -= 1
            if skw_waitime == 0:
                # クールダウン（待機時間）が終わったら状態を完全にリセット
                skwww = ""
                skwtime = 0
                
        if skw_waitime < 0 :
            skw_waitime = 0

        sideTrace()
        
        # utime.sleep_ms(5)
except Exception as e:
    print("\nError:", e)
finally:
    o.stop()
    hcsr.deactivate()
    print("\nfinished?")
    led.off()