from machine import I2C
from bno08x import *
class angler:
    def __init__(self,BNO:BNO08X) -> None:
        self.BNO085 = BNO
        self.BNO085.tare()

    def yaw(self):
        _, _, yaw = self.BNO085.euler
        return yaw

    def turn_deg_move(self,deg):
        yaw = self.yaw()

    def face_deg_move(self,deg):
        pass

    def turn_deg_omega(self,deg):
        pass

    def face_deg_omega(self,deg):
        pass