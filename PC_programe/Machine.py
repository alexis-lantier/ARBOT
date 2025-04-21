from Ball import Ball
from Plate import Plate
from Cam import Cam
from Connection_Cam import ConnectionCam


class Machine:

    def __init__(self):
        self.Ball = Ball(cam=Cam(connectionCam=ConnectionCam()))
        self.Plate = Plate()

    def GetAnglePhi(self):
        pass

    def GetAngleTheta(self):
        pass

    def GetHeight(self):
        pass

    def RegulationCenter(self):
        pass

    def RegulationBounce(self):
        pass
