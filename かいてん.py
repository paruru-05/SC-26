print("Hello!")
from machine import Pin, PWM
from utime import sleep
from bno08x import *
from MotorConfig import *
import omni
import mcp3008
import rp2
import time
import math
import random

led = Pin("LED", Pin.OUT)

led.on()

print("start")

o = omni.Omni(PWM_LIST,IN_2_LIST,IN_1_LIST)

sleep(1)
goal = 90
try:
    while True:
        if rp2.bootsel_button() == 1:
            raise ValueError("bootsel")

        led.on()

        o.move(1,0,5000)
                
        # steps = 360

        # for i in range(steps):
        # # 0 〜 2*pi で1周
        #     t = (i / steps) * (2 * math.pi)

        #     # 位置の微分（速度ベクトル）
        #     Vx = math.cos(t)
        #     Vy = math.cos(2 * t)  # ここが cos(2t) になることで途中で符号が反転する！
        #     # クラスのmoveメソッドを呼び出し
        #     o.move(Vx=Vx, Vy=Vy, power=20000, omega=0)

        #     time.sleep_ms(20)

        # for deg in range(360):
        #     rad = math.radians(deg)
            
        #     Vx = v_target * math.cos(rad)
        #     Vy = v_target * math.sin(rad)
        #     o.move(0,-1,20000,-0.2)
        #     time.sleep_ms(10)
except Exception as e:
    print(e)
finally:
    o.stop()
    led.off()

    print("finished?")