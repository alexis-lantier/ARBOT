from Ball import Ball
from Plate import Plate
from Cam import Cam
import math
import time


class Machine:

    def __init__(self, port='COM5', baudrate=115200):
        
        self._ball = Ball() 
        self._plate = Plate( port=port, baudrate=baudrate)
        self._bounceAutorised = True
        self._last_bounce_time = 0
        self._min_bounce_interval = 0.2  # Intervalle minimum entre les rebonds en secondes
        self._bounce_offset = 0.1  # Temps d'avance pour déclencher le rebond avant l'impact

        self._virtualAnglePhi=0
        self._virtualAngleTheta=0

    def GetAnglePhi(self):
        # Get the angle phi from the Ball object
        return self._plate.GetAnglePhi()

    def GetAngleTheta(self):
        
        return self._plate.GetAngleTheta()

    def GetHeight(self):
        
        return self._plate.GetHeight()

    def RegulationCenter(self):
        self._virtualAngleTheta,self._virtualAnglePhi = self.calculate_angles()
       
        # if 0.05 < abs(self._plate._angleTheta-angle_teta) :
        #     self._plate.MoveAxisTheta(angle_teta)
        # if 0.05< abs(self._plate._anglePhi-angle_phi) :
        #     self._plate.MoveAxisPhi(angle_phi)

    def calculate_fall_time(self, height, velocity):
        """
        Calcule le temps estimé pour atteindre une certaine hauteur depuis la position actuelle,
        en tenant compte de la vitesse verticale initiale.
        - height: position verticale actuelle (en mm, par rapport au sol/caméra)
        - velocity: vitesse verticale actuelle (en mm/s, positive vers le haut)
        Retourne le temps (en secondes) pour que la balle atteigne la hauteur 0 (le sol).
        """
        g = 9810  # gravité en mm/s²
        if height <= 0:
            return None  # La balle est déjà en dessous du sol
        a = 0.5 * g
        b = -velocity
        c = -height
        discriminant = b * b - 4 * a * c
        if discriminant < 0:
            return None  # Pas de solution réelle
        sqrt_disc = math.sqrt(discriminant)
        t1 = (-b + sqrt_disc) / (2 * a)
        t2 = (-b - sqrt_disc) / (2 * a)
        # On retourne la plus grande solution positive (si la balle monte, c’est celle-ci qu’il faut)
        times = [t for t in (t1, t2) if t > 0]
        return max(times) if times else None

    def RegulationBounce(self):
        if self._ball._cam._radius is None:
            return

        current_time = time.time()
        z = self._ball._cam._position.z
        vz = self._ball._cam._ballSpeed.z
        d = self._ball._cam._radius

        # Vérification de l'intervalle minimum entre les rebonds
        if current_time - self._last_bounce_time < self._min_bounce_interval:
            return

        # Détection par hauteur et vitesse
        fall_time = self.calculate_fall_time(z, vz)
        if vz < 0:
            
            if self._bounceAutorised:
                # Cas 1: La balle est déjà très basse
                if z < 100:
                    print(f"💥 Rebond forcé ! (Hauteur critique atteinte: {z:.1f}mm)")
                    self._plate.MakeOneBounce(self._virtualAngleTheta, self._virtualAnglePhi)
                    self._bounceAutorised = False
                    self._last_bounce_time = current_time
                    return
                    
                # Cas 2: Détection par temps de chute
                elif fall_time is not None and fall_time < self._bounce_offset:
                    print(f"💥 Rebond déclenché ! (Temps de chute: {fall_time:.3f}s, Offset: {self._bounce_offset:.3f}s)")
                    self._plate.MakeOneBounce(self._virtualAngleTheta, self._virtualAnglePhi)
                    self._bounceAutorised = False
                    self._last_bounce_time = current_time
                    return                   

                # Détection par diamètre (solution de secours)
                if z < 325 and self._bounceAutorised:
                    print("💥 Rebond déclenché par diamètre !")
                    self.RegulationCenter()
                    self._plate.MakeOneBounce(self._virtualAngleTheta, self._virtualAnglePhi)
                    self._bounceAutorised = False
                    self._last_bounce_time = current_time
                    return

        # Réautorisation du rebond si la balle est assez haute
        if z > 200 and not self._bounceAutorised:
            self._bounceAutorised = True

        print(f"Position Z: {z:.1f}mm, Vitesse Z: {vz:.1f}mm/s, Rayon: {d:.1f}px")

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
 
        # Initialisation de l'historique si ce n'est pas déjà fait
        if not hasattr(self, "_vx_history"):
            self._vx_history = [0.0] * 2
            self._vy_history = [0.0] * 2
            self._history_index = 0
 
        # Normalisation de z
        z_normalisé = max(0.0, min(1.0, 1 - z / z_max))+ 0.3
 
        # Mise à jour de l'historique des vitesses
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
        Kp = 0.015
        Kd = 0.025  # Gain dérivé vitesse
        Kp=0
 
        # Régulation PD
        theta = - (Kp * ex + Kd * avg_vx) 
        phi   =   (Kp * ey + Kd * avg_vy) 
 
        # Clamp pour éviter les dépassements
        theta = max(-angle_max, min(angle_max, theta))
        phi   = max(-angle_max, min(angle_max, phi))
        return theta, phi



