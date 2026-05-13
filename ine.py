from bno08x import *
from i2c import BNO08X_I2C
from machine import SoftI2C, Pin
import utime

# --- 設定 ---
# ピン番号はあなたの環境(SCL=5, SDA=4, INT=6)に合わせています
i2c0 = SoftI2C(scl=Pin(5), sda=Pin(4), freq=50000) # 50kHzに下げて安定性を最大に
int_pin = Pin(6, Pin.IN, Pin.PULL_UP)

print("--- Step 1: Power cycle check ---")
print("Now, please unplug and replug the BNO's 3.3V wire.")

# INTピンが0になる（BNOが起動する）のを待つ
while int_pin.value() == 1:
    utime.sleep_ms(10)

print("BNO Detected! Waiting for it to settle...")
utime.sleep_ms(500) # 起動直後の不安定な時間をあえて待つ

# --- Step 2: 初期化 ---
print("--- Step 2: Initializing ---")
try:
    # reset_pin=None にすることで、ソフトリセット命令を極力スキップさせる
    bno = BNO08X_I2C(i2c0, address=0x4a, int_pin=int_pin, reset_pin=None)
    print("Successfully connected!")
except Exception as e:
    print(f"Still getting error: {e}")
    print("Try to proceed without full reset...")
    # ここでもし bno が作成できていれば、以下の処理に進む

# --- Step 3: 強制レポート有効化 ---
if 'bno' in locals():
    try:
        # BNOが黙らないように、ゆっくり一つずつ有効化する
        bno.acceleration.enable()
        utime.sleep_ms(200)
        print("Acceleration enabled.")
    except:
        print("Failed to enable acceleration.")

    while True:
        try:
            bno.update_sensors()
            if bno.acceleration.updated:
                print("Accel:", bno.acceleration)
        except Exception as e:
            # OSError 5 や 110 が出ても無視してリトライ
            pass
        utime.sleep_ms(20)