import rp2
from machine import Pin
import time

Pin(10,Pin.OUT).on()
time.sleep(0.1)

@rp2.asm_pio(set_init=rp2.PIO.OUT_LOW,)
def hcsr04_program():
    wrap_target()
    set(pins, 1)      
    set(x, 19)        
    mov(isr, x)       
    in_(null, 6)      
    mov(x, isr)       
    label("delay1")
    jmp(x_dec, "delay1")
    set(pins, 0)      
    wait(1, pin, 0)    
    mov(x, invert(null)) 
    label("timer")
    jmp(x_dec, "test")   
    jmp("timerstop")     # 
    label("test")
    jmp(pin, "timer")
    label("timerstop")
    mov(isr, invert(x))  
    push(noblock)
    set(x, 28)           
    mov(isr, x)          
    in_(null, 18)        
    mov(x, isr)          
    label("delay2")
    jmp(x_dec, "delay2") # 
    wrap()

TRIG_PIN = 15
ECHO_PIN = 14

sm = rp2.StateMachine(
    0, 
    hcsr04_program, 
    freq=125_000_000, 
    set_base=Pin(TRIG_PIN), 
    in_base=Pin(ECHO_PIN),
    jmp_pin=Pin(ECHO_PIN)
)

sm.active(1)

print("計測を開始します...")

try:
    while True:
        if sm.rx_fifo():
            raw_value = sm.get()
            distance = (raw_value * 2 / 125_000_000) * 34300 / 2
            print(f"距離: {distance:.2f} cm")
        time.sleep(0)
except KeyboardInterrupt:
    sm.active(0)
    print("止まりました")