from machine import PWM as _PWM
from machine import Pin as _Pin
from time import ticks_ms as _ticks_ms

GRN  = "\033[32m"
YEL  = "\033[33m"
CYA  = "\033[36m"
RST  = "\033[0m"
BLD  = "\033[1m"
DIM  = "\033[2m"
_pf = "[MOTOR]"
_msg = lambda _msg,end="\n",_pfenable=True:print(f"{RST}{BLD}{YEL}[{_ticks_ms() / 1000:>8.3f}]{_pf if _pfenable else ""}{RST} {_msg}{RST}",end=end)

def init_motor(freq:int = 10000):
    print(RST+BLD+GRN+"="*22+"MOTOR"+"="*23)
    _msg("Initializing Motor Drivers...")
    
    FAULT = _Pin(11, _Pin.IN, _Pin.PULL_UP)
    SLEEP = _Pin(10, _Pin.OUT)
    SLEEP.off()

    #driver 1
    PH_1 = _Pin(0, _Pin.OUT)
    EN_1 = _PWM(_Pin(1))

    #driver 2
    PH_2 = _Pin(2, _Pin.OUT)
    EN_2 = _PWM(_Pin(3))

    #driver 3
    PH_3 = _Pin(4, _Pin.OUT)
    EN_3 = _PWM(_Pin(5))

    #driver 4
    PH_4 = _Pin(6, _Pin.OUT)
    EN_4 = _PWM(_Pin(7))

    _msg("Initializing PWM...")

    EN_LIST =[EN_1,EN_2,EN_3,EN_4]
    PH_LIST = [PH_1,PH_2,PH_3,PH_4]
    _msg("Initialized:"," ")
    _msg(PH_LIST)
    for EN,name in zip(EN_LIST,["EN_A","EN_B","EN_C","EN_D\n"]):
        EN.freq(freq)
        EN.duty_u16(0)
        _msg(YEL+name,"",False)

    SLEEP.on()
    
    print(BLD+GRN+"="*50+RST)