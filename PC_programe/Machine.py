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

        z = self._ball._cam._position.z
        vz = self._ball._cam._ballSpeed.z
        d = 2 * self._ball._cam._radius  # Diamètre de la balle
        anticipation = 0.20
        g = -9810

        # Méthode 1 : Calcul physique (prioritaire)
        z_plaque = self._plate.GetHeight() - self._ball._cam._radius
        a = 0.5 * g
        b = vz
        c = z - z_plaque
        delta = b**2 - 4*a*c
        if delta >= 0 and a != 0:
            t1 = (-b - math.sqrt(delta)) / (2*a)
            t2 = (-b + math.sqrt(delta)) / (2*a)
            temps_impact = min(t for t in [t1, t2] if t > 0) if any(t > 0 for t in [t1, t2]) else float('inf')
        else:
            temps_impact = float('inf')

        if temps_impact < anticipation and self._bounceAutorised:
            self._bounceAutorised = False
            self._plate.MakeOneBounce()
        # Méthode 2 : Rebond simple basé sur le diamètre (seulement si la physique n'a pas déclenché)
        elif d > 45 and self._bounceAutorised:
            print("💥 Rebond déclenché par diamètre !")
            self._plate.MakeOneBounce()
            self._bounceAutorised = False

        # Réautorisation du rebond si la balle est assez haute
        if z >= 365.6:
            self._bounceAutorised = True

        delta = 0.2  # contre reaction experimentale certifiée par la norme ISO B.R.I.C.O.L.A.G.E
        self._plate._height = self._plate._height - delta 
        self._plate._axisA._height = self._plate._axisA._height - delta
        self._plate._axisB._height = self._plate._axisB._height - delta
        self._plate._axisC._height = self._plate._axisC._height - delta

    

    def calculate_angles(self):
        """
        Calcule les angles theta et phi à partir de la position (erreur) et de la vitesse (PD)
        """
        z_max = 365.6
        z = self._ball._cam._position.z

        # Si z est None, ne rien faire
        if self._ball._cam._radius is None:
            return 0, 0

        # Erreur de position (distance au centre)
        ex = self._ball._cam._position.x
        ey = self._ball._cam._position.y

        # Vitesse
        vx = self._ball._cam._ballSpeed.x
        vy = self._ball._cam._ballSpeed.y
        angle_max = 30.0

        # Initialisation de l'historique si ce n’est pas déjà fait
        if not hasattr(self, "_vx_history"):
            self._vx_history = [0.0] * 2
            self._vy_history = [0.0] * 2
            self._history_index = 0

        # Normalisation de z
        z_normalisé = max(0.0, min(1.0, 1 - z / z_max))

        # Mise à jour de l’historique des vitesses
        self._vx_history[self._history_index] = vx
        self._vy_history[self._history_index] = vy
        self._history_index = (self._history_index + 1) % 2

        # Moyenne glissante
        avg_vx = sum(self._vx_history) / 2
        avg_vy = sum(self._vy_history) / 2

        # Gains à ajuster selon ton système
        Kp = 0.002   # Gain proportionnel position
        Kd = 0.005  # Gain dérivé vitesse

        # Régulation PD
        theta = - (Kp * ex + Kd * avg_vx) * z_normalisé
        phi   =   (Kp * ey + Kd * avg_vy) * z_normalisé

        # Clamp pour éviter les dépassements
        theta = max(-angle_max, min(angle_max, theta))
        phi   = max(-angle_max, min(angle_max, phi))

        return theta, phi