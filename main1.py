print("Hello!")
from machine import Pin, PWM
from utime import sleep
import math

led = Pin("LED", Pin.OUT)

led.on()

print("setting pins...")
#driver 1
pwma = PWM(Pin(2))
ain2 = Pin(3, Pin.OUT)
ain1 = Pin(4, Pin.OUT)
vcc = "VCC"
stby = Pin(5, Pin.OUT)
gnd = "GND"
bin1 = Pin(6, Pin.OUT)
bin2 = Pin(7, Pin.OUT)
pwmb = PWM(Pin(8))

#driver 2
pwmc = PWM(Pin(10))
cin2 = Pin(11, Pin.OUT)
cin1 = Pin(12, Pin.OUT)
vcc = "VCC"
stby2 = Pin(22, Pin.OUT)
gnd = "GND"
din1 = Pin(21, Pin.OUT)
din2 = Pin(20, Pin.OUT)
pwmd = PWM(Pin(19))

pwms=[pwmb,pwmc,pwma,pwmd]
in1s=[bin1,cin1,ain1,din1]
in2s=[bin2,cin2,ain2,din2]

print("setting PWM...")
pwma.freq(10000)
pwma.duty_u16(0)
pwmb.freq(10000)
pwmb.duty_u16(0)
pwmc.freq(10000)
pwmc.duty_u16(0)
pwmd.freq(10000)
pwmd.duty_u16(0)

def brake():
    ain1.on()
    ain2.on()
    bin1.on()
    bin2.on()
    cin1.on()
    cin2.on()
    din1.on()
    din2.on()

def forward():
    ain1.off()
    ain2.on()
    bin1.on()
    bin2.off()
    cin1.off()
    cin2.on()
    din1.on()
    din2.off()
    
def reverse():
    ain1.on()
    ain2.off()
    bin1.off()
    bin2.on()
    cin1.on()
    cin2.off()
    din1.off()
    din2.on()

def free():
    ain1.off()
    ain2.off()
    bin1.off()
    bin2.off()
    cin1.off()
    cin2.off()
    din1.off()
    din2.off()
    
def turn(pwm:PWM,in1:Pin,in2:Pin,speed:int,brake:bool=True):
    speed = int(speed)
    absp = min(30000,abs(speed))
    if speed < 0:
        in1.off()
        in2.on()
    elif speed > 0:
        in1.on()
        in2.off()
    else:
        if brake:
            in1.on()
            in2.on()
        else:
            in1.off()
            in2.off()
    pwm.duty_u16(absp)
    
def omni(Vx:float,Vy:float,power:int=25000,brake:bool=True):
    speeds = [
        (Vx+Vy)*power,
        (Vx-Vy)*power,
        (-Vx-Vy)*power,
        (-Vx+Vy)*power
    ]
    for i in range(4):
        turn(pwms[i],in1s[i],in2s[i],speeds[i],brake)

led.off()

print("start")

led.on()

stby.on()
stby2.on()

for degree in range(0, 360, 10):
    rad = math.radians(degree)
    vx = math.cos(rad)
    vy = math.sin(rad)
    
    print(f"角度: {degree}, Vx: {vx:.2f}, Vy: {vy:.2f}")
    omni(vx, vy,power=20000)
    sleep(0.1)

led.off()
stby.off()
stby2.off()
pwma.deinit()
pwmb.deinit()
pwmc.deinit()
pwmd.deinit()

print("finished?")