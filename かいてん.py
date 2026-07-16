print("Hello!")
from machine import Pin, PWM
from utime import sleep
from bno08x import *
import omni
import mcp3008
import rp2
import time
import angler
import math
import random

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


pwms=[pwma,pwmb,pwmc,pwmd]
in1s=[ain1,bin1,cin1,din1]
in2s=[ain2,bin2,cin2,din2]

o = omni.Omni(pwms,in1s,in2s)

def is_bootsel_pressed():
    return rp2.bootsel_button() == 1
print("Wait for BOOTSEL...")
# メインループの前にボタンを待つ


print("setting PWM...")
pwma.freq(10000)
pwmb.freq(10000)
pwmb.duty_u16(30000)
pwmc.freq(10000)
pwmc.duty_u16(30000)
pwmd.freq(10000)
pwmd.duty_u16(30000)

led.off()

print("start")

led.on()

stby.on()
stby2.on()

print("BNO08x I2C connection : Done\n")

last = None
shikii = 30
cpt = 0
average_delay = -1

sleep(1)
goal = 90
try:
    while True:
        if rp2.bootsel_button() == 1:
            raise ValueError("bootsel")

        led.on()
        o.move(0,0,omega=1)
        pwma.duty_u16(30000)
        time.sleep(2)
        o.stop()
        time.sleep(2)
        led.off()
        o.move(0,0,omega=1)
        time.sleep(2)
        o.stop()
        time.sleep(2)
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