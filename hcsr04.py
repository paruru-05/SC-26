import rp2
from machine import Pin
from re import sub

class HCSR04:
    def __init__(self, machine_no:int,trig:Pin,echo:Pin) -> None:
        print("\033[36m\033[1m"+"="*22+"HCSR04"+"="*22)
        print("\033[36m\033[1m[HCSR04]\033[0m Initializing HCSR04...")
        self._machine_no = machine_no
        self._TRIG = trig
        self._ECHO = echo
        print(f"\033[36m\033[1m[HCSR04]\033[0m Info\033[0m TRIG_PIN: \033[2mPin{sub(r"\D", "", str(trig))[:-1]}\033[0m  ECHO_PIN: \033[2mPin{sub(r"\D", "", str(echo))[:-1]}\033[0m  MACHINE_NO: \033[2m{machine_no}\033[0m")
        self._distance_value = 0
        self._raw = 0
        self._sm = rp2.StateMachine(
        self._machine_no,
        self._hcsr04_program, 
        freq=125_000_000, 
        set_base=self._TRIG, 
        in_base=self._ECHO,
        jmp_pin=self._ECHO
        )
        print("\033[36m\033[1m[HCSR04]\033[0m\033[1m Initialize Complete.\033[0m")
        print("\033[36m\033[1m"+"="*50+"\033[0m")

    def distance(self):
        self._distance_value = (self.raw_value() * 2 / 125_000_000) * 34300 / 2
        return self._distance_value

    def raw_value(self):
        if self._sm.rx_fifo():
            self._raw = self._sm.get()
        return self._raw

    def activate(self):
        self._sm.active(1)

    def deactivate(self):
        self._sm.active(0)
    
    @staticmethod
    @rp2.asm_pio(set_init=rp2.PIO.OUT_LOW,)
    def _hcsr04_program():
        wrap_target()           #type: ignore
        set(pins, 1)            #type: ignore    
        set(x, 19)              #type: ignore
        mov(isr, x)             #type: ignore
        in_(null, 6)            #type: ignore
        mov(x, isr)             #type: ignore
        label("delay1")         #type: ignore
        jmp(x_dec, "delay1")    #type: ignore
        set(pins, 0)            #type: ignore
        wait(1, pin, 0)         #type: ignore
        mov(x, invert(null))    #type: ignore
        label("timer")          #type: ignore
        jmp(x_dec, "test")      #type: ignore
        jmp("timerstop")        #type: ignore
        label("test")           #type: ignore
        jmp(pin, "timer")       #type: ignore
        label("timerstop")      #type: ignore
        mov(isr, invert(x))     #type: ignore
        push(noblock)           #type: ignore
        set(x, 28)              #type: ignore
        mov(isr, x)             #type: ignore
        in_(null, 18)           #type: ignore
        mov(x, isr)             #type: ignore
        label("delay2")         #type: ignore
        jmp(x_dec, "delay2")    #type: ignore
        wrap()                  #type: ignore