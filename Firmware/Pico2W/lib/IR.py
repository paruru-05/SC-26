
from machine import Pin as _Pin
from machine import ADC as _ADC
import math as _math

IR_ADC_A_RAW = _ADC(_Pin(28))
IR_ADC_B_RAW = _ADC(_Pin(27))

IR_S0 = _Pin(4, _Pin.OUT)
IR_S1 = _Pin(5, _Pin.OUT)
IR_S2 = _Pin(6, _Pin.OUT)
IR_EN = _Pin(7, _Pin.OUT)

class IR_RING:

    _IR_COS = (
        1.0, 0.923880, 0.707107, 0.382683,
        0.0, -0.382683, -0.707107, -0.923880,
        -1.0, -0.923880, -0.707107, -0.382683,
        0.0, 0.382683, 0.707107, 0.923880
    )

    _IR_SIN = (
        0.0, 0.382683, 0.707107, 0.923880,
        1.0, 0.923880, 0.707107, 0.382683,
        0.0, -0.382683, -0.707107, -0.923880,
        -1.0, -0.923880, -0.707107, -0.382683
    )

    def __init__(self):
        IR_EN.off()

    def enable(self):
        IR_EN.on()

    def disable(self):
        IR_EN.off()

    def select_ch(self, ch: int):
        IR_S0.value(ch & 1)
        IR_S1.value((ch >> 1) & 1)
        IR_S2.value((ch >> 2) & 1)

    def read_ch(self, ch: int):
        self.select_ch(ch)
        a = IR_ADC_A_RAW.read_u16()
        b = IR_ADC_B_RAW.read_u16()
        return (a, b)

    def read_all(self):
        results = [0] * 16

        select = self.select_ch
        adc_a = IR_ADC_A_RAW
        adc_b = IR_ADC_B_RAW

        for ch in range(8):
            select(ch)
            results[ch] = adc_a.read_u16()
            results[ch + 8] = adc_b.read_u16()

        return results

    def get_direction(self, robot_angle: float = 0):
        select = self.select_ch
        adc_a = IR_ADC_A_RAW
        adc_b = IR_ADC_B_RAW

        cos = self._IR_COS
        sin = self._IR_SIN

        x = 0.0
        y = 0.0
        total = 0

        for ch in range(8):
            select(ch)

            a = adc_a.read_u16()
            b = adc_b.read_u16()

            x += a * cos[ch] + b * cos[ch + 8]
            y += a * sin[ch] + b * sin[ch + 8]

            total += a + b

        if total == 0:
            return None

        direction = (
            robot_angle - _math.degrees(_math.atan2(y, x))
        ) % 360

        return (direction, total)
