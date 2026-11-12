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

    _IR_COS = (1.0, 0.923880, 0.707107, 0.382683, 0.0, -0.382683, -0.707107, -0.923880, -1.0, -0.923880, -0.707107, -0.382683, 0.0, 0.382683, 0.707107, 0.923880)
    _IR_SIN = (0.0, 0.382683, 0.707107, 0.923880, 1.0, 0.923880, 0.707107, 0.382683, 0.0, -0.382683, -0.707107, -0.923880, -1.0, -0.923880, -0.707107, -0.382683)

    def __init__(self) -> None:
        IR_EN.off()

    def enable(self):
        IR_EN.on()

    def disable(self):
        IR_EN.off()

    def select_ch(self,ch:int):
        s0 = (ch >> 0) & 1
        s1 = (ch >> 1) & 1
        s2 = (ch >> 2) & 1
        IR_S0.value(s0)
        IR_S1.value(s1)
        IR_S2.value(s2)

    def read_ch(self,ch:int):
        self.select_ch(ch)
        return (IR_ADC_A_RAW.read_u16(), IR_ADC_B_RAW.read_u16())

    def read_all(self):
        results = [0] * 16
        for ch in range(8):
            r1, r2 = self.read_ch(ch)
            results[ch] = r1
            results[ch + 8] = r2
        return results

    def get_direction(self, robot_angle:float=0):
        results = self.read_all()
        x = 0.0
        y = 0.0
        total = 0

        for i in range(16):
            strength = results[i]
            x += strength * self._IR_COS[i]
            y += strength * self._IR_SIN[i]
            total += strength
        if total == 0:
            return None
        direction = (robot_angle - _math.degrees(_math.atan2(y, x))) % 360
        return (direction, total)