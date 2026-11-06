from machine import Pin
import machine
import time

in_pin = Pin(15, Pin.IN, Pin.PULL_DOWN)
out_pin = Pin(16, Pin.OUT)
while True:
    out_pin.toggle()
    print(in_pin.value())
    time.sleep(0.5)