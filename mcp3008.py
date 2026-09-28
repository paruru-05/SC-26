import machine
class mcp:
    def __init__(self,spi:machine.SPI,cs:machine.Pin) -> None:
        self.spi = spi
        self.cs = cs
        cs.on()
        print("\033[1m"+"\033[33m"+"="*23+"MCP3008"+"="*20+"\033[0m")
    def read_adc(self, channel=0):
        # 1バイト目: 0x01 (Start bit)
        # 2バイト目: (8 + channel) << 4
        #   -> 8 (10002) = SGL=1, D2=0, D1=0, D0=0 
        #   -> << 4 で上位4ビット(MSB側)へ配置: [SGL][D2][D1][D0][0][0][0][0]
        self.cs.off()
        tx = bytes([0x01, (8 + channel) << 4, 0x00])
        rx = bytearray(3)
        self.spi.write_readinto(tx, rx)
        self.cs.on()
        # 10ビットデータの復元
        # rx[1] の下位2ビット（D9, D8） + rx[2]（D7〜D0）
        return ((rx[1] & 0x03) << 8) | rx[2]