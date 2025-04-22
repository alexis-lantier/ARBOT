from Connection_Motor import ConnectionMotor
from Axis import Axis


class Plate:
    def __init__(self):
        self.anglePhi = 0
        self._angleTheta = 0
        self._height = 0
        self._ConnectionMotor = ConnectionMotor()
        self._axisA = Axis()
        self._axisB = Axis()
        self._axisC = Axis()

    def MoveAxisHeigh(self, height):
        # monter tous les axes
        self._axisA.move(height)
        self._axisB.move(height)
        self._axisC.move(height)

        self._ConnectionMotor.SendMMovement(
            self._axisA._motor._lastAngleMovement,
            self._axisB._motor._lastAngleMovement,
            self._axisC._motor._lastAngleMovement,
        )

    def MoveAxisPhi(self, angle):
        # 3 inconues
        # 3 equations
        #  1. Angle Theta
        #  2. Angle Phi
        #  3. Height constante

        #  ANGLE -->  HAUTEUR DE CHAQUE AXIS -->  angle MOTEUR(FONNCTION FAITES)
        # TRANSIMITION ANGLE AU MOTTEURS connection.

        self._ConnectionMotor.SendMMovement(
            self._axisA._motor._lastAngleMovement,
            self._axisB._motor._lastAngleMovement,
            self._axisC._motor._lastAngleMovement,
        )

    def MoveAxisTheta(self, angle):

        # 3 inconues
        # 3 equations
        #  1. Angle Theta
        #  2. Angle Phi
        #  3. Height constante

        # changer la valeur des moteurs
        # envoyer l'information à la carte de faire bouger
        self._ConnectionMotor.SendMMovement(
            self._axisA._motor._lastAngleMovement,
            self._axisB._motor._lastAngleMovement,
            self._axisC._motor._lastAngleMovement,
        )

    def GetAnglePhi(self):
        return self.anglePhi

    def GetAngleTheta(self):
        return self._angleTheta

    def GetHeight(self):
        return self._height
