from machine import PWM,Pin

RED  = "\033[31m"
GRN  = "\033[32m"
YEL  = "\033[33m"
BLU  = "\033[34m"
CYA  = "\033[36m"
RST  = "\033[0m"
BLD  = "\033[1m"
DIM  = "\033[2m"
pf = "[MOTOR]"

msg = lambda msg,end="\n",pfenable=True:print(f"{RST}{BLD}{GRN}{pf if pfenable else ""}{RST} {msg}{RST}",end=end)

print(RST+BLD+GRN+"="*22+"MOTOR"+"="*23)

msg("Initializing Motor Drivers...")

#driver 1
msg(DIM+"Initializing "+CYA+"Driver 1..."," ")
PWM_A = PWM(Pin(21))
PWM_B = PWM(Pin(18))
msg(DIM+YEL+"PWM_A/B"," ",False)
IN_A_1 = Pin(19, Pin.OUT)
IN_A_2 = Pin(20, Pin.OUT)
msg(DIM+RED+"IN_A"," ",False)
IN_B_1 = Pin(17, Pin.OUT)
IN_B_2 = Pin(16, Pin.OUT)
msg(DIM+RED+"IN_B"," ",False)
STBY_1 = Pin(22, Pin.OUT)
msg(DIM+BLU+"STBY",pfenable=False)

msg("Initialized "+CYA+BLD+"Driver 1")

#driver 2
msg(DIM+"Initializing "+CYA+"Driver 2..."," ")
PWM_C = PWM(Pin(7))
PWM_D = PWM(Pin(12))
msg(DIM+YEL+"PWM_C/D"," ",False)
IN_C_2 = Pin(8, Pin.OUT)
IN_C_1 = Pin(9, Pin.OUT)
msg(DIM+RED+"IN_C"," ",False)
IN_D_1 = Pin(10, Pin.OUT)
IN_D_2 = Pin(11, Pin.OUT)
msg(DIM+RED+"IN_D"," ",False)
STBY_2 = Pin(13, Pin.OUT)
msg(DIM+BLU+"STBY",pfenable=False)

msg("Initialized "+CYA+BLD+"Driver 2")

msg("Initializing PWM...")

PWM_LIST =[PWM_A,PWM_B,PWM_C,PWM_D]
IN_1_LIST = [IN_A_1,IN_B_1,IN_C_1,IN_D_1]
IN_2_LIST = [IN_A_2,IN_B_2,IN_C_2,IN_D_2]
msg("Initialized:"," ")
for pwm,name in zip(PWM_LIST,["PWM_A","PWM_B","PWM_C","PWM_D\n"]):
    pwm.freq(10000)
    pwm.duty_u16(0)
    msg(YEL+name,"",False)
msg(BLD+"Initialize Complete.")
print(BLD+GRN+"="*50+RST)