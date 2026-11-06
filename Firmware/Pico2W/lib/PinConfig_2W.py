from machine import Pin as _Pin
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