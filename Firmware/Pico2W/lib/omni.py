from machine import Pin, PWM

class Omni:

    def __init__(self, ens:list, phs:list,enable:bool=True) -> None:
        self._ens = ens
        self._phs = phs
        self.MOTOR_ENABLE = enable

    def motor(self,en:PWM,ph:Pin,speed:float|int,brake:bool=True):
        speed = int(speed)
        absp = min(60000,abs(speed))
        if speed < 0:
            ph.off()
        elif speed > 0:
            ph.on()
        en.duty_u16(absp)
        
    def move(self,Vx:float,Vy:float,power:int=25000,omega:int|float=0,brake:bool=True):
        speeds = [
            ( Vx + Vy + omega) * power,
            (-Vx + Vy - omega) * power,
            (-Vx + Vy + omega) * power,
            ( Vx + Vy - omega) * power
        ]
        if self.MOTOR_ENABLE:
            for i in range(4):
                self.motor(self._ens[i],self._phs[i],speeds[i],brake)
            
    def stop(self):
        for i in range(4):
            self.motor(self._ens[i],self._phs[i],0,False)
        
    
    def __del__(self):
        self.stop()