from machine import Pin
import time
from hcsr04 import HCSR04
import omni
from MotorConfig import *

TRIG_PIN = 14
ECHO_PIN = 15
LED_PIN = Pin("LED")

hcsr = HCSR04(0,Pin(14),Pin(15))

time.sleep(0.5)

print("計測を開始します...")

hcsr.activate()

o = omni.Omni(PWM_LIST,IN_1_LIST,IN_2_LIST)
o.move(0,1,10000,0.5)
time.sleep(1)
o.stop()

try:
    while True:
        dis = hcsr.distance()
        print(f"距離: {dis:.4f} cm")
        if dis < 20:
            o.move(0,1,5000)
        else:
            o.move(0,-1,10000)
        time.sleep(0.05)
except KeyboardInterrupt:
    o.stop()
    print("止まりました")