from typing import Any

from machine import Pin as _Pin
from machine import ADC as _ADC
from machine import I2C as _I2C
from time import ticks_ms as _ticks_ms

GRN  = "\033[32m"
YEL  = "\033[33m"
CYA  = "\033[36m"
RST  = "\033[0m"
BLD  = "\033[1m"
DIM  = "\033[2m"
_pf = "[OTHER_PIN]"
_msg = lambda _msg,end="\n",_pfenable=True:print(f"{RST}{BLD}{YEL}[{_ticks_ms() / 1000:>8.3f}]{_pf if _pfenable else ""}{RST} {_msg}{RST}",end=end)

_msg("PUSH_SW")
PUSH_SW = _Pin(18, _Pin.IN)

_msg("SLIDER")
SLIDER_LEFT = _Pin(20, _Pin.IN)
SLIDER_RIGHT = _Pin(19, _Pin.IN)
SLIDER_STATUS = SLIDER_LEFT.value

_msg("LED")
LED_M = _Pin("LED", _Pin.OUT)

LED_1 = _Pin(13, _Pin.OUT)
LED_2 = _Pin(14, _Pin.OUT)
LED_3 = _Pin(16, _Pin.OUT)
LED_4 = _Pin(17, _Pin.OUT)

_msg("Buzzer")
from buzzer import *
BUZZER = BZR()

_msg("I2C")
I2C = _I2C(0, scl=_Pin(1), sda=_Pin(0), freq=400000)

_msg("BNO_INTN")
BNO_INTN = _Pin(2, _Pin.IN)

_msg("VBAT_ADC")
VBAT_ADC_RAW = _ADC(_Pin(26))
def VBAT_VOLTAGE():
    _ = VBAT_ADC_RAW.read_u16()
    return VBAT_ADC_RAW.read_u16() * 0.00023685392

_msg("IR")
from IR import *
IR = IR_RING()
IR.enable()

_msg("UART")
from UART_2W import *
uart = Pico2WUART()