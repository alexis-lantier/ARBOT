from Connection_Motor import ConnectionMotor
from Axis import Axis


class Plate:
    def __init__(self):
        self.anglePhi = 0
        self._angleTheta = 0
        self._height = 0
        self._ConnectionMotor = ConnectionMotor.ConnectionMotor()
        self._axisA = Axis()
        self._axisB = Axis()
        self._axisC = Axis()

    def MoveAxisHeight(self, height):
        # changer la valeur des moteurs
        # envoyer l'information à la carte de faire bouger
        pass

    def MoveAxisPhi(self, angle):
        # changer la valeur des moteurs
        # envoyer l'information à la carte de faire bouger
        pass

    def MoveAxisTheta(self, angle):
        # changer la valeur des moteurs
        # envoyer l'information à la carte de faire bouger
        pass
