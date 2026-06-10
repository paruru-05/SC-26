print("Hello!")
from machine import Pin, PWM, I2C
from utime import sleep
from bno08x import *
import omni
import rp2
import time
import angler
import math
import random

I2C1_SDA = Pin(0)
I2C1_SCL = Pin(1)

i2c1 = I2C(0, scl=I2C1_SCL, sda=I2C1_SDA, freq=100000, timeout=200000)

led = Pin("LED", Pin.OUT)

led.on()

print("setting pins...")
#driver 1
pwma = PWM(Pin(21))
ain2 = Pin(20, Pin.OUT)
ain1 = Pin(19, Pin.OUT)
vcc = "VCC"
stby = Pin(22, Pin.OUT)
gnd = "GND"
bin1 = Pin(17, Pin.OUT)
bin2 = Pin(16, Pin.OUT)
pwmb = PWM(Pin(18))

#driver 2
pwmc = PWM(Pin(7))
cin2 = Pin(8, Pin.OUT)
cin1 = Pin(9, Pin.OUT)
vcc = "VCC"
stby2 = Pin(13, Pin.OUT)
gnd = "GND"
din1 = Pin(10, Pin.OUT)
din2 = Pin(11, Pin.OUT)
pwmd = PWM(Pin(12))

# PIOプログラムの定義
@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW,)
def hcsr04_program():
    wrap_target()
    # Triggerピンを1にする
    set(pins, 1)      
    
    # 10usのディレイ
    set(x, 19)        
    mov(isr, x)       
    in_(null, 6)      
    mov(x, isr)       
    label("delay1")
    jmp(x_dec, "delay1") # 

    # Triggerピンを0に戻す
    set(pins, 0)      

    # EchoピンがHighになるのを待つ
    wait(1, pin, 0)    

    # タイマー開始 (0xFFFFFFFF)
    mov(x, invert(null)) 
    
    label("timer")
    # カウントダウン
    jmp(x_dec, "test")   
    jmp("timerstop")     # 
    
    label("test")
    # EchoピンがHighの間ループ
    jmp(pin, "timer")   

    label("timerstop")
    # 反転させてパルス幅（時間）を計算
    mov(isr, invert(x))  
    # FIFOにプッシュ
    push(noblock)
    # 60msの待機
    set(x, 28)           
    mov(isr, x)          
    in_(null, 18)        
    mov(x, isr)          
    label("delay2")
    jmp(x_dec, "delay2") # 
    wrap()

TRIG_PIN = 15
ECHO_PIN = 14

# ステートマシンの初期化 (125MHz) 
# sm = rp2.StateMachine(
#     0, 
#     hcsr04_program, 
#     freq=125_000_000, 
#     set_base=Pin(TRIG_PIN), 
#     in_base=Pin(ECHO_PIN),  # wait や in で使うベースピン [cite: 32, 40]
#     jmp_pin=Pin(ECHO_PIN)   # これで 'jmp pin' 命令が ECHO_PIN を見るようになります 
# )


pwms=[pwma,pwmb,pwmc,pwmd]
in1s=[ain1,bin1,cin1,din1]
in2s=[ain2,bin2,cin2,din2]

o = omni.Omni(pwms,in1s,in2s)

def is_bootsel_pressed():
    return rp2.bootsel_button() == 1
print("Wait for BOOTSEL...")
# メインループの前にボタンを待つ
while True:
    if is_bootsel_pressed():
        print("BOOTSEL pressed.")
        break
    time.sleep(0.1)

#sm.active(1)
# def distanceget():
#     # FIFOから読み取った値は2クロックサイクル単位
#     # 距離(cm) = (カウント * 2 / 125,000,000) * 34300 / 2
#     raw_value = sm.get()
#     distance = (raw_value * 2 / 125_000_000) * 34300 / 2
#     print(f"距離: {distance:.5f} cm")
#     return distance

print("setting PWM...")
pwma.freq(10000)
pwma.duty_u16(0)
pwmb.freq(10000)
pwmb.duty_u16(0)
pwmc.freq(10000)
pwmc.duty_u16(0)
pwmd.freq(10000)
pwmd.duty_u16(0)

led.off()

print("start")

led.on()

stby.on()
stby2.on()

bno = BNO08X(i2c1, debug=False)
print("BNO08x I2C connection : Done\n")

bno.enable_feature(BNO_REPORT_ACCELEROMETER, 20)
bno.enable_feature(BNO_REPORT_MAGNETOMETER,20 )
bno.enable_feature(BNO_REPORT_GYROSCOPE,20 )
bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR, 10)
bno.set_quaternion_euler_vector(BNO_REPORT_GAME_ROTATION_VECTOR)
last = None
shikii = 30
cpt = 0
average_delay = -1

sleep(0.01)
bno.tare()
goal = 90
try:
    while True:
        if rp2.bootsel_button() == 1:
            raise ValueError("bootsel")
        _, _, yaw = bno.euler

        for deg in range(0,1000,1):
            Vx = 0
            Vy = 0.001*deg
            o.move(Vx,Vy,25000,0,False)
            time.sleep_ms(3)
        for deg in range(1000,0,-1):
            Vx = 0
            Vy = 0.001*deg
            o.move(Vx,Vy,25000,0,False)
            time.sleep_ms(3)
        for deg in range(0,1000,1):
            Vx = 0
            Vy = 0.001*deg
            o.move(Vx,-Vy,25000,0,False)
            time.sleep_ms(3)
        for deg in range(1000,0,-1):
            Vx = 0
            Vy = 0.001*deg
            o.move(Vx,-Vy,25000,0,False)
            time.sleep_ms(3)

        # if yaw > goal:
        #     pwr = int((goal - yaw)*250)
        #     o.move(0,0,pwr,-1)
        # elif yaw < goal:
        #     pwr = int((goal- yaw)*250)
        #     o.move(0,0,pwr,1)


        # distance = distanceget()
        # if distance > shikii:
        #     if last == "far":
        #         vx = 1
        #         vy = 0
        #         o.move(vx, vy,power=5000)
        #     else:
        #         vx = 0
        #         vy = 0
        #         o.move(vx, vy,power=5000)
        #         time.sleep_ms(5)
        #     last = "far"
        # elif distance < shikii:
        #     if last == "near":
        #         vx = -1
        #         vy = 0
        #         o.move(vx, vy,power=5000)
        #     else:
        #         vx = 0
        #         vy = 0
        #         o.move(vx, vy,power=5000)
        #         time.sleep_ms(5)
        #     last = "near"
except Exception as e:
    print(e)
finally:
    o.stop()
    led.off()
    stby.off()
    stby2.off()
    pwma.deinit()
    pwmb.deinit()
    pwmc.deinit()
    pwmd.deinit()

    print("finished?")