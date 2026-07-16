import machine
class mcp:
    def __init__(self,spi:machine.SPI,cs:machine.Pin) -> None:
        self.spi = spi
        self.cs = cs
        cs.on()

    def read_adc(self,channel=0):
        # 送信データの作成 (スタートビット、チャンネル指定、ダミー)
        tx = bytes([0x01, (8 + channel) << 4, 0x00])
        rx = bytearray(3)
        self.spi.write_readinto(tx, rx)
        print(rx)
        return ((rx[1] & 0x03) << 8) | rx[2]