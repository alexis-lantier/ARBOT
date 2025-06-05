from Ball import Ball
from Plate import Plate
from Cam import Cam
import math
import time

#########################################################

class BallPredictor:
    def __init__(self, offsetmot=0.1):
        self.g = 9810  # mm/s²
        self.offsetmot = offsetmot
        self.predictions = []
        self.activation_done = False
 
    def calculate_fall_time(self, height, velocity):
        if height <= 0:
            return None
        a = 0.5 * self.g
        b = -velocity
        c = -height
        discriminant = b * b - 4 * a * c
        if discriminant < 0:
            return None
        sqrt_disc = math.sqrt(discriminant)
        t1 = (-b + sqrt_disc) / (2 * a)
        t2 = (-b - sqrt_disc) / (2 * a)
        times = [t for t in (t1, t2) if t > 0]
        return max(times) if times else None
 
    def add_prediction(self, height, velocity):
        now = time.time()
        t_chute = self.calculate_fall_time(height, velocity)
        if t_chute is not None:
            t_final = now + t_chute
            self.predictions.append(t_final)
            if len(self.predictions) > 5:
                self.predictions.pop(0)
 
    def get_activation_time(self):
        if not self.predictions:
            return None
        t_f_moyen = sum(self.predictions) / len(self.predictions)
        return t_f_moyen - self.offsetmot
 
    def should_activate_motor(self):
        if self.activation_done:
            return False
        t_activation = self.get_activation_time()
        if t_activation and time.time() >= t_activation:
            self.activation_done = True
            return True
        return False
 
    def reset(self):
        self.predictions.clear()
        self.activation_done = False



##################################################################



class Machine:

    def __init__(self, port='COM5', baudrate=115200):
        
        self._ball = Ball() 
        self._plate = Plate( port=port, baudrate=baudrate)
        self._bounceAutorised = True
        self._last_bounce_time = 0
        self._min_bounce_interval = 0.2  # Intervalle minimum entre les rebonds en secondes
        self._bounce_offset = 0.3  # Temps d'avance pour déclencher le rebond avant l'impact

        self._virtualAnglePhi=0
        self._virtualAngleTheta=0

        self._predictor = BallPredictor(offsetmot=self._bounce_offset)

    def GetAnglePhi(self):
        # Get the angle phi from the Ball object
        return self._plate.GetAnglePhi()

    def GetAngleTheta(self):
        
        return self._plate.GetAngleTheta()

    def GetHeight(self):
        
        return self._plate.GetHeight()

    def RegulationCenter(self):
        self._virtualAngleTheta,self._virtualAnglePhi = self.calculate_angles()
       

   

    def RegulationBounce(self):
        if self._ball._cam._radius is None:
            return

        current_time = time.time()
        z = self._ball._cam._position.z
        
        vz = self._ball._cam._ballSpeed.z
        if abs(vz) < 20:
            vz = 0

        d = self._ball._cam._radius

        
 
        if vz < 0:
            self._predictor.add_prediction(z, vz)
            if self._bounceAutorised:
                # Cas 1: La balle est très basse
                if(0):
                    if z < 100:
                        print(f"💥 Rebond forcé ! (Hauteur critique atteinte: {z:.1f}mm)")
                        self._plate.MakeOneBounce(self._virtualAngleTheta, self._virtualAnglePhi)
                        self._bounceAutorised = False
                        self._last_bounce_time = current_time
                        self._predictor.reset()
                        return
 
                # Cas 2: Prédiction basée sur plusieurs hauteurs
                if(1):
                        if self._predictor.should_activate_motor():
                            print(f"💥 Rebond déclenché par prédiction multiple !")
                            self._plate.MakeOneBounce(self._virtualAngleTheta, self._virtualAnglePhi)
                            self._bounceAutorised = False
                            self._last_bounce_time = current_time
                            self._predictor.reset()
                        return
 
                # Cas 3: Détection de secours par diamètre
                if(0):
                    if z < 290:
                        print("💥 Rebond déclenché par diamètre !")
                        self.RegulationCenter()
                        self._plate.MakeOneBounce(self._virtualAngleTheta, self._virtualAnglePhi)
                        self._bounceAutorised = False
                        self._last_bounce_time = current_time
                        self._predictor.reset()
                        return

        
        # Réautorisation du rebond si la balle est assez haute
        if z > 100 and not self._bounceAutorised:
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
        Kp = 0.05
        Kd = 0.05  # Gain dérivé vitesse
        Kd=0
        
 
        # Régulation PD
        theta = - (Kp * ex + Kd * avg_vx) 
        phi   =   (Kp * ey + Kd * avg_vy) 
 
        # Clamp pour éviter les dépassements
        theta = max(-angle_max, min(angle_max, theta))
        phi   = max(-angle_max, min(angle_max, phi))
        return theta, phi



