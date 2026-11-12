from machine import Pin
from machine import Pin

# ピンの初期設定
LED_M = Pin("LED", Pin.OUT)
LED_1 = Pin(13, Pin.OUT)
LED_2 = Pin(14, Pin.OUT)
LED_3 = Pin(16, Pin.OUT)
LED_4 = Pin(17, Pin.OUT)

# LEDをリストにまとめる
leds = [LED_1, LED_2, LED_3, LED_4]

PUSH_SW = Pin(18, Pin.IN)

SLIDER_LEFT = Pin(20, Pin.IN)
SLIDER_RIGHT = Pin(19, Pin.IN)
SLIDER_STATUS = SLIDER_LEFT.value

for led in leds:
    led.off()
LED_M.off()

while True:
    if SLIDER_RIGHT.value():
        LED_2.on()
    else:
        LED_2.off()
    if PUSH_SW.value():
        LED_1.on()
    else:
        LED_1.off()