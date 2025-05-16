from Ball import Ball
from Plate import Plate
from Cam import Cam



class Machine:

    def __init__(self):
        
        self._ball = Ball() 
        self._plate = Plate()
        self._bounceAutorised = True
        
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
            if self._ball._cam._radius is None:
                return
            if self._ball._cam._radius > 22 and self._bounceAutorised:
                self._bounceAutorised = False
                self._plate.MakeOneBounce()

            elif self._ball._cam._radius <= 18:
                self._bounceAutorised = True
