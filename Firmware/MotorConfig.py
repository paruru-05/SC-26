from machine import PWM,Pin

GRN  = "\033[32m"
YEL  = "\033[33m"
CYA  = "\033[36m"
RST  = "\033[0m"
BLD  = "\033[1m"
DIM  = "\033[2m"
pf = "[MOTOR]"

msg = lambda msg,end="\n",pfenable=True:print(f"{RST}{BLD}{GRN}{pf if pfenable else ""}{RST} {msg}{RST}",end=end)

print(RST+BLD+GRN+"="*22+"MOTOR"+"="*23)

msg("Initializing Motor Drivers...")

SLEEP = Pin(10)
SLEEP.off()

#driver 1
msg(DIM+"Initializing "+CYA+"Driver 1..."," ")
PH_1 = Pin(0)
EN_1 = PWM(Pin(1))

#driver 2
msg(DIM+"Initializing "+CYA+"Driver 2..."," ")
PH_2 = Pin(2)
EN_2 = PWM(Pin(3))

#driver 3
msg(DIM+"Initializing "+CYA+"Driver 3..."," ")
PH_3 = Pin(4)
EN_3 = PWM(Pin(5))

#driver 4
msg(DIM+"Initializing "+CYA+"Driver 4..."," ")
PH_4 = Pin(6)
EN_4 = PWM(Pin(7))

msg("Initializing PWM...")

EN_LIST =[EN_1,EN_2,EN_3,EN_4]
PH_LIST = [PH_1,PH_2,PH_3,PH_4]
msg("Initialized:"," ")
for EN,name in zip(EN_LIST,["EN_A","EN_B","EN_C","EN_D\n"]):
    EN.freq(10000)
    EN.duty_u16(0)
    msg(YEL+name,"",False)

SLEEP.on()

msg(BLD+"Initialize Complete.")
print(BLD+GRN+"="*50+RST)