print("Hello!")
from machine import Pin, SPI
import utime
from bno08x import *
import mcp3008
import rp2

Pin(5).off()

spi = SPI(0,sck=Pin(2), mosi=Pin(3), miso=Pin(4),polarity=0, phase=0,baudrate=10000)
#sck=Pin(2), mosi=Pin(3), miso=Pin(4)
mcp = mcp3008.mcp(spi,Pin(5))
Pin(5).on()

led = Pin("LED", Pin.OUT)

led.on()

def is_bootsel_pressed():
    return rp2.bootsel_button() == 1

led.off()

print("start")

led.on()

utime.sleep(1)

def p(v,l=20,c=0):
 n=max(0,min(1,v/1000))
 f=int(n*l)
 print(v)
 print(f"{'\r' if c == 0 else ''}[{'|'*f}{' '*(l-f)}] {int(n*100)}%",end="")

try:
    while True:
        if rp2.bootsel_button() == 1:
            raise ValueError("bootsel")
        p(mcp.read_adc(0),25,0)
        # p(mcp.read_adc(1),25,1)
        # p(mcp.read_adc(2),25,2)
except Exception as e:
    print(e)
finally:
    led.off()
    print("finished?")