from machine import Pin,PWM
le = Pin(0)
pwm = PWM(le)
sw = Pin(28, Pin.IN, Pin.PULL_UP)
duty = 0
pwm.freq(1000)

while True:
    if not sw.value():
        if duty < 65535:
            duty += 2
    else:
        if duty > 0:
            duty -= 2
    pwm.duty_u16(duty)