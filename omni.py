from machine import Pin, PWM

class Omni:

    def __init__(self, pwms:list, in1s:list, in2s:list,enable:bool=True) -> None:
        self._pwms = pwms
        self._in1s = in1s
        self._in2s = in2s
        self.MOTOR_ENABLE = enable

    def motor(self,pwm:PWM,in1:Pin,in2:Pin,speed:float|int,brake:bool=True):
        speed = int(speed)
        absp = min(30000,abs(speed))
        assert absp <= 30000
        if speed < 0:
            in1.off()
            in2.on()
        elif speed > 0:
            in1.on()
            in2.off()
        else:
            if brake:
                in1.on()
                in2.on()
            else:
                in1.off()
                in2.off()
        pwm.duty_u16(absp)
        
    def move(self,Vx:float,Vy:float,power:int=25000,omega:int|float=0,brake:bool=True):
        speeds = [
            ( Vx + Vy + omega) * power,
            (-Vx + Vy - omega) * power,
            (-Vx + Vy + omega) * power,
            ( Vx + Vy - omega) * power
        ]
        if self.MOTOR_ENABLE:
            for i in range(4):
                self.motor(self._pwms[i],self._in1s[i],self._in2s[i],speeds[i],brake)
            
    def stop(self):
        for i in range(4):
            self.motor(self._pwms[i],self._in1s[i],self._in2s[i],0,False)
        
    
    def __del__(self):
        self.stop()

