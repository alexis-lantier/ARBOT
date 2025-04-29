from Ball import Ball
from Plate import Plate
from Cam import Cam
from Connection_Cam import ConnectionCam


class Machine:

    def __init__(self):
        
        self._ball = Ball(cam=Cam(connectionCam=ConnectionCam())) 
        self._plate = Plate()
        
    def GetAnglePhi(self):
        # Get the angle phi from the Ball object
        return self._plate.GetAnglePhi()

    def GetAngleTheta(self):
        
        return self._plate.GetAngleTheta()

    def GetHeight(self):
        
        return self._plate.GetHeight()

    def RegulationCenter(self):
        pass

    def RegulationBounce(self):
        pass
