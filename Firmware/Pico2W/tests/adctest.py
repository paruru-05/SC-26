import machine
adc = machine.ADC(28)

while True:
    print(adc.read_u16() * 3.3 / 65535)