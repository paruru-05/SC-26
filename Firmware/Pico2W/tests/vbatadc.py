from machine import ADC, Pin
import time

print("=== VBAT_ADC TEST ===")

vbat_adc = ADC(Pin(26))

while True:
    raw = vbat_adc.read_u16()
    voltage = raw * 0.00023685392

    print("RAW:", raw, " VBAT:", round(voltage, 3), "V")
    time.sleep_ms(500)
