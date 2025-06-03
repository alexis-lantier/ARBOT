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
        if 0.05 < abs(self._plate._angleTheta-angle_teta) :
            self._plate.MoveAxisTheta(angle_teta)
        if 0.05< abs(self._plate._anglePhi-angle_phi) :
            self._plate.MoveAxisPhi(angle_phi)

    def RegulationBounce(self):
        if self._ball._cam._radius is None:
            return

        d = self._ball._cam._radius
        z = self._ball._cam._position.z
       
        if z < 325 and self._bounceAutorised:
            print("💥 Rebond déclenché par diamètre !")
            self.RegulationCenter()
            self._plate.MakeOneBounce()
            self._bounceAutorised = False
            

        # Réautorisation du rebond si la balle est assez haute
        if z > 325:
            self._bounceAutorised = True


        delta = 0.2  
        self._plate._height = self._plate._height - delta 
        self._plate._axisA._height = self._plate._axisA._height - delta
        self._plate._axisB._height = self._plate._axisB._height - delta
        self._plate._axisC._height = self._plate._axisC._height - delta
        return self._bounceAutorised

    

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
        z_normalisé = max(0.0, min(1.0, 1 - z / z_max))+ 0.3
 
        # Mise à jour de l’historique des vitesses
        self._vx_history[self._history_index] = vx
        self._vy_history[self._history_index] = vy
        self._history_index = (self._history_index + 1) % 2
 
        # Moyenne glissante
        avg_vx = sum(self._vx_history) / 2
        avg_vy = sum(self._vy_history) / 2

        if 1:
            avg_vx = vx
            avg_vy = vy
 
        # Gains à ajuster selon ton système
        Kp = 0.025
        Kd = 0.015  # Gain dérivé vitesse
 
        # Régulation PD
        theta = - (Kp * ex + Kd * avg_vx) 
        phi   =   (Kp * ey + Kd * avg_vy) 
 
        # Clamp pour éviter les dépassements
        theta = max(-angle_max, min(angle_max, theta))
        phi   = max(-angle_max, min(angle_max, phi))

        print (f"x {ex} y {ey} ")
 
        return theta, phi
    


    