from Ball import Ball
from Plate import Plate
from Cam import Cam
import math


class Machine:

    def __init__(self, port='COM5', baudrate=115200):
        
        self._ball = Ball() 
        self._plate = Plate( port=port, baudrate=baudrate)
        self._bounceAutorised = True
        
    def GetAnglePhi(self):
        # Get the angle phi from the Ball object
        return self._plate.GetAnglePhi()

    def GetAngleTheta(self):
        
        return self._plate.GetAngleTheta()

    def GetHeight(self):
        
        return self._plate.GetHeight()

    def RegulationCenter(self):
        angle_teta,angle_phi = self.calculate_angles()
        self._plate.MoveAxisTheta(angle_teta)
        self._plate.MoveAxisPhi(angle_phi)

    def RegulationBounce(self):
            if self._ball._cam._radius is None:
                return
            if self._ball._cam._position.z > 293 and self._bounceAutorised:
                self._bounceAutorised = False
                self._plate.MakeOneBounce()

            elif self._ball._cam._position.z <= 365.6:
                self._bounceAutorised = True

    

    def calculate_angles(self):
        """
        vx, vy : vitesses de la balle
        z : hauteur actuelle de la balle
        z_max : hauteur maximale attendue de la balle (normalisation)
        angle_max : angle maximum autorisé pour le plateau (en degrés)
        """

        # Normalisation de z pour avoir un facteur entre 0 (haut) et 1 (sol)
        z_max = 365.6
        z = self._ball._cam._position.z
        vx = self._ball._cam._ballSpeed.x
        vy = self._ball._cam._ballSpeed.y
        angle_max = 30.0

        z_normalisé = 0.5 #max(0.0, min(1.0, 1 - z / z_max))

        # Facteur de gain sur l'inclinaison
        facteur = angle_max * z_normalisé

        # Calcul directionnel des angles selon les vitesses
        theta = math.degrees(math.atan(vx)) * z_normalisé
        phi = math.degrees(math.atan(vy)) * z_normalisé

        # Clamp les angles à ±angle_max
        theta = max(-angle_max, min(angle_max, theta))
        phi = max(-angle_max, min(angle_max, phi))

        return theta, phi
