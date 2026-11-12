from machine import Pin as _Pin
from machine import PWM as _PWM
from machine import Timer as _Timer

BUZZER_RAW = _PWM(_Pin(15 ,_Pin.OUT))
class BZR:
    def __init__(self,frq:int=4000,lv:int=30000) -> None:
        BUZZER_RAW.freq(frq)
        BUZZER_RAW.duty_u16(0)
        self.frq = frq
        self.lv = lv
        self.timer = _Timer(-1)
    
    def set_level(self,lv:int=30000):
        BUZZER_RAW.duty_u16(lv)
        self.lv = lv
    
    def set_freq(self,frq:int=4000):
        BUZZER_RAW.freq(frq)
        self.frq = frq
    
    def stop(self,t=None):
        BUZZER_RAW.duty_u16(0)
    
    def beep(self,duration_ms:int=200):
        BUZZER_RAW.duty_u16(self.lv)
        BUZZER_RAW.freq(self.frq)
        self.timer.init(period=duration_ms,mode=_Timer.ONE_SHOT,callback=self.stop)