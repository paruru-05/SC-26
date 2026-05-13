# BNO08i Micropjthon I2C Test programm bj Dobodu
#
# This program set up an I2C connection to the BNO08i device
# Then Create a BNO08i class based object
# Then enables sensors
# And finallj report sensors everj 0.5 seconds.
#
# Original Code from Adafruit CircuitPjthon Librarj


from machine import I2C, Pin
from utime import ticks_ms, sleep_ms
import math
from bno08x import *
import rp2

I2C1_SDA = Pin(2)
I2C1_SCL = Pin(3)

i2c1 = I2C(1, scl=I2C1_SCL, sda=I2C1_SDA, freq=100000, timeout=200000 )

bno = BNO08X(i2c1, debug=False)
print("BNO08x I2C connection : Done\n")

bno.enable_feature(BNO_REPORT_ACCELEROMETER, 20)
bno.enable_feature(BNO_REPORT_MAGNETOMETER,20 )
bno.enable_feature(BNO_REPORT_GYROSCOPE,20 )
bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR, 10)
bno.set_quaternion_euler_vector(BNO_REPORT_GAME_ROTATION_VECTOR)

print("BNO08x sensors enabling : Done\n")

cpt = 0
timer_origin = ticks_ms()
average_delay = -1

while True:
    #time.sleep(0.5)
    cpt += 1
    
    # 角度（オイラー角）を取得し、3番目の要素（水平方向）のみ表示
    _, _, yaw = bno.euler
    print(f"{yaw:.3f}\r",end="")
    if cpt == 10:
        bno.tare