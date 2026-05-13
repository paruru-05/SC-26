from bno08x import *
from i2c import BNO08X_I2C
from machine import SoftI2C, Pin
import utime

# 1. I2Cの初期化 (SoftI2Cで安定化)
i2c0 = SoftI2C(scl=Pin(5), sda=Pin(4), freq=20000)

# スキャン結果の表示
print("I2C scan:", [hex(a) for a in i2c0.scan()])

# 2. ピンの設定
int_pin = Pin(6, Pin.IN, Pin.PULL_UP)
reset_pin = None 

# --- 起動待ち処理を追加 ---
print("Waiting for BNO08x to wake up (INT to go Low)...")
# 3.3V抜き差しで0が来るタイミングを捕まえるために、少しの間待機します
start_time = utime.ticks_ms()
while int_pin.value() == 1:
    if utime.ticks_diff(utime.ticks_ms(), start_time) > 5000: # 5秒待機
        print("Timeout: INT stayed High. Try replugging BNO 3.3V now...")
        break
    utime.sleep_ms(10)

# 3. BNO08xの初期化 (エラーが出ても続行を試みる)
try:
    bno = BNO08X_I2C(i2c0, address=0x4a, int_pin=int_pin, reset_pin=reset_pin)
    print("BNO08x initialized!")
except Exception as e:
    print(f"Initialization error: {e}")
    print("Trying to proceed anyway...")
    # 初期化に失敗してもbnoオブジェクトが作られていれば続行
# -----------------------

# レポートの有効化
try:
    bno.acceleration.enable()
    bno.gyro.enable()
    bno.quaternion.enable()
except:
    print("Could not enable reports. Check connection.")

while True:
    try:
        # センサーデータの更新を確認
        bno.update_sensors()
        
        # 加速度の取得
        if bno.acceleration.updated:
            accel_x, accel_y, accel_z = bno.acceleration
            print(f"Accel: {accel_x:.2f}, {accel_y:.2f}, {accel_z:.2f}")

        # 姿勢角の取得
        if bno.quaternion.updated:
            yaw, pitch, roll = bno.quaternion.euler
            print(f"Yaw: {yaw:.2f}, Pitch: {pitch:.2f}, Roll: {roll:.2f}")
            
    except Exception as e:
        # 通信エラーが出てもループを止めない
        pass
    
    utime.sleep_ms(10)