from machine import Pin as _Pin
from machine import PWM as _PWM
from machine import UART as _UART
from time import ticks_ms as _ticks_ms

GRN  = "\033[32m"
YEL  = "\033[33m"
CYA  = "\033[36m"
RST  = "\033[0m"
BLD  = "\033[1m"
DIM  = "\033[2m"
_pf = "[OTHER_PIN]"
_msg = lambda _msg,end="\n",_pfenable=True:print(f"{RST}{BLD}{YEL}[{_ticks_ms() / 1000:>8.3f}]{_pf if _pfenable else ""}{RST} {_msg}{RST}",end=end)

def init_other_pins():
    _msg("Initializing Other Pins...")

    _msg("DRIBBLER_PWM")
    DRIBBLER = _PWM(_Pin(12, _Pin.OUT))
    DRIBBLER.freq(10000)
    DRIBBLER.duty_u16(0)

    _msg("KICK")
    KICK = _Pin(15, _Pin.OUT, value=0)

    _msg("SuperSonic")
    _msg("TRIG")
    FRONT_TRIG = _Pin(16, _Pin.OUT)
    LEFT_TRIG = _Pin(18, _Pin.OUT)
    RIGHT_TRIG = _Pin(20, _Pin.OUT)
    REAR_TRIG = _Pin(22, _Pin.OUT)
    _msg("ECHO")
    FRONT_ECHO = _Pin(14, _Pin.IN, _Pin.PULL_DOWN)
    LEFT_ECHO = _Pin(19, _Pin.IN, _Pin.PULL_DOWN)
    RIGHT_ECHO = _Pin(21, _Pin.IN, _Pin.PULL_DOWN)
    REAR_ECHO = _Pin(26, _Pin.IN, _Pin.PULL_DOWN)

    _msg("UART")
    UART = _UART(1, baudrate=9600, tx=_Pin(8), rx=_Pin(9))