from machine import Pin, PWM
import time,random

# Pin(6) を周波数 50Hz の PWM として初期化
servo = PWM(Pin(16), freq=50)
servo2 = PWM(Pin(17), freq=50)

def set_angle(angle):
    # 角度をパルス幅 (ns) に変換
    pulse_ns = int(500000 + (angle / 180) * 2000000)
    servo.duty_ns(pulse_ns)

def set_angle2(angle):
    # 角度をパルス幅 (ns) に変換
    pulse_ns = int(500000 + (angle / 180) * 2000000)
    servo2.duty_ns(pulse_ns)

# set_angle2(180)
# set_angle(100)

set_angle2(80)
set_angle(10)

# 2　出口
#無印　奥

for i in range(40):
    set_angle2(180)
    set_angle(100)
    set_angle2(180)
    set_angle(100)
    time.sleep(0.5)
    set_angle2(80)
    set_angle(20)
    set_angle2(80)
    set_angle(20)
    time.sleep(0.5)


time.sleep(0.3)

servo.deinit()