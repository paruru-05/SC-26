print("Hello!")
from machine import Pin, PWM, I2C, SPI
from utime import sleep
from bno08x import *
import omni
import mcp3008
import rp2
import time
import angler
import math
import random

I2C1_SDA = Pin(0)
I2C1_SCL = Pin(1)

i2c1 = I2C(0, scl=Pin(1), sda=Pin(0), freq=100000)
spi = SPI(0,sck=Pin(2), mosi=Pin(3), miso=Pin(4))
mcp = mcp3008.mcp(spi,Pin(5))
bno = BNO08X(i2c1, debug=False)

print("BNO08x I2C connection : Done\n")

bno.enable_feature(BNO_REPORT_ACCELEROMETER, 20)
bno.enable_feature(BNO_REPORT_MAGNETOMETER,20 )
bno.enable_feature(BNO_REPORT_GYROSCOPE,20 )
bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR, 10)
bno.set_quaternion_euler_vector(BNO_REPORT_GAME_ROTATION_VECTOR)

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
while True:
    if is_bootsel_pressed():
        print("BOOTSEL pressed.")
        break
    time.sleep(0.1)

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

sleep(1)
bno.tare()
goal = 90
_, _, yaw = bno.euler
try:
    while True:
        if rp2.bootsel_button() == 1:
            raise ValueError("bootsel")

        print(mcp.read_adc())

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