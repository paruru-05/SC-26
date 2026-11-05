import usb.device
from usb.device.keyboard import KeyboardInterface
import machine
import time
# 初期化
kbd = KeyboardInterface()
usb.device.get().init(kbd, builtin_driver=True)
machine.Pin("LED").on()
time.sleep(3)
# 送信 (例: "hello" と入力する)
kbd.send_string("HELLO")
machine.Pin("LED").off()