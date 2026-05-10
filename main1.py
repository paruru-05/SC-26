print("Hello!")
from machine import Pin, PWM
from utime import sleep
import omni
import rp2
import time


led = Pin("LED", Pin.OUT)

led.on()

print("setting pins...")
#driver 1
pwma = PWM(Pin(19))
ain2 = Pin(20, Pin.OUT)
ain1 = Pin(21, Pin.OUT)
vcc = "VCC"
stby = Pin(3, Pin.OUT)
gnd = "GND"
bin1 = Pin(2, Pin.OUT)
bin2 = Pin(1, Pin.OUT)
pwmb = PWM(Pin(0))

#driver 2
pwmc = PWM(Pin(12))
cin2 = Pin(17, Pin.OUT)
cin1 = Pin(16, Pin.OUT)
vcc = "VCC"
stby2 = Pin(9, Pin.OUT)
gnd = "GND"
din1 = Pin(10, Pin.OUT)
din2 = Pin(11, Pin.OUT)
pwmd = PWM(Pin(18))

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
sm = rp2.StateMachine(
    0, 
    hcsr04_program, 
    freq=125_000_000, 
    set_base=Pin(TRIG_PIN), 
    in_base=Pin(ECHO_PIN),  # wait や in で使うベースピン [cite: 32, 40]
    jmp_pin=Pin(ECHO_PIN)   # これで 'jmp pin' 命令が ECHO_PIN を見るようになります 
)


pwms=[pwmb,pwmc,pwma,pwmd]
in1s=[bin1,cin1,ain1,din1]
in2s=[bin2,cin2,ain2,din2]

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

sm.active(1)
def distanceget():
    # FIFOから読み取った値は2クロックサイクル単位
    # 距離(cm) = (カウント * 2 / 125,000,000) * 34300 / 2
    raw_value = sm.get()
    distance = (raw_value * 2 / 125_000_000) * 34300 / 2
    print(f"距離: {distance:.5f} cm")
    return distance

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

sleep(0.5)
last = None
shikii = 30
try:
    while True:
        if rp2.bootsel_button() == 1:
            raise ValueError("bootsel")
        distance = distanceget()
        if distance > shikii:
            if last == "far":
                vx = 1
                vy = 0
                o.move(vx, vy,power=5000)
            else:
                vx = 0
                vy = 0
                o.move(vx, vy,power=5000)
                time.sleep_ms(5)
            last = "far"
        elif distance < shikii:
            if last == "near":
                vx = -1
                vy = 0
                o.move(vx, vy,power=5000)
            else:
                vx = 0
                vy = 0
                o.move(vx, vy,power=5000)
                time.sleep_ms(5)
            last = "near"
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