from machine import Pin, PWM

class Omni:

    def __init__(self, pwms:list, in1s:list, in2s:list) -> None:
        self.pwms = pwms
        self.in1s = in1s
        self.in2s = in2s

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
        
    def move(self,Vx:float,Vy:float,power:int=25000,brake:bool=True):
        speeds = [
            (Vx+Vy)*power,
            (Vx-Vy)*power,
            (-Vx-Vy)*power,
            (-Vx+Vy)*power
        ]
        for i in range(4):
            self.motor(self.pwms[i],self.in1s[i],self.in2s[i],speeds[i],brake)
            
    def stop(self):
        for i in range(4):
            self.motor(self.pwms[i],self.in1s[i],self.in2s[i],0,False)
        
    
    def __del__(self):
        self.stop()

