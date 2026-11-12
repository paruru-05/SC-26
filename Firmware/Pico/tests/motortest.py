
from machine import PWM as _PWM
from machine import Pin as _Pin
from time import sleep_ms

FAULT = _Pin(11, _Pin.IN, _Pin.PULL_UP)
SLEEP = _Pin(10, _Pin.OUT)
PH_4 = _Pin(6, _Pin.OUT)
EN_4 = _PWM(_Pin(7))

PWM_FREQ = 10000
DUTY_MAX = 20000

SLEEP.off()
EN_4.freq(PWM_FREQ)
EN_4.duty_u16(0)
PH_4.value(0)

print("=== MOTOR 4 TEST ===")
print("FAULT:", FAULT.value())

SLEEP.on()
sleep_ms(100)

def motor(duty, direction):
    duty = max(0, min(DUTY_MAX, duty))
    PH_4.value(direction)
    EN_4.duty_u16(duty)
    print("DIR:", direction, "DUTY:", duty,
          "FAULT:", FAULT.value())

try:
    while True:
        # 正転：段階的に速度上昇
        print("\n=== FORWARD RAMP ===")
        for duty in range(0, DUTY_MAX + 1, 5000):
            motor(duty, 0)
            sleep_ms(1000)

        motor(0, 0)
        sleep_ms(1500)

        # 逆転：段階的に速度上昇
        print("\n=== REVERSE RAMP ===")
        for duty in range(0, DUTY_MAX + 1, 5000):
            motor(duty, 1)
            sleep_ms(1000)

        motor(0, 0)
        sleep_ms(2000)

except KeyboardInterrupt:
    print("\nTest stopped")

finally:
    EN_4.duty_u16(0)
    PH_4.value(0)
    SLEEP.off()
    EN_4.deinit()
    print("Motor driver disabled.")
