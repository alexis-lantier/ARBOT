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
        self._bounce_offset = 0.1 # Temps d'avance pour déclencher le rebond avant l'impact

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
       

    def calculate_fall_time(self, height, velocity):
        """
        Calcule le temps de chute estimé en fonction de la hauteur et de la vitesse.
        Retourne le temps en secondes avant impact au sol, ou None si non calculable.
        """
        g = 9810  # Accélération gravitationnelle en mm/s²
        
        # Protection contre les valeurs aberrantes
        if height is None or velocity is None:
            return None
        
        # Si la balle monte ou si hauteur négative, pas de calcul
        if velocity >= 0 or height <= 0:
            return None
        
        # Ajout d'une hauteur minimale pour éviter les erreurs de calcul
        height = max(10.0, height)
        
        # Équation du second degré pour le temps de chute: h = h0 + v0*t + (1/2)*g*t²
        a = g/2
        b = velocity
        c = height
        
        # Calcul du discriminant
        discriminant = b*b - 4*a*c
        if discriminant < 0:
            return None
            
        # Calcul des solutions
        t1 = (-b + math.sqrt(discriminant)) / (2*a)
        t2 = (-b - math.sqrt(discriminant)) / (2*a)
        
        # On prend la solution positive la plus petite
        if t1 > 0 and t2 > 0:
            return min(t1, t2)
        elif t1 > 0:
            return t1
        elif t2 > 0:
            return t2
        
        return None

#modif
    def RegulationBounce(self):
        """
        Détecte quand la balle va toucher le sol et déclenche un rebond au moment optimal.
        Utilise plusieurs méthodes de détection pour une meilleure fiabilité.
        """
        # Vérification que la balle est bien détectée
        if self._ball._cam._radius is None:
            return
        
        # Récupération des données actuelles de la balle
        current_time = time.time()
        z = self._ball._cam._position.z
        vz = self._ball._cam._ballSpeed.z
        
        # Protection contre les valeurs aberrantes
        if z is None or vz is None or z < 0:  # Protection contre les hauteurs négatives
            return
        
        # Vérification de l'intervalle minimum entre les rebonds
        if current_time - self._last_bounce_time < self._min_bounce_interval:
            return
        
        # Paramètres de configuration
        critical_height = 100  # mm, hauteur critique pour forcer un rebond
        bounce_trigger_time = 0.15  # secondes avant impact pour déclencher le rebond
        
        # Réautorisation du rebond si la balle est assez haute
        if z > 200:
            self._bounceAutorised = True
        
        # Si les rebonds sont autorisés
        if self._bounceAutorised:
            # Détection par hauteur et vitesse (méthode principale)
            print(f"Position actuelle: z={z:.1f}mm, vitesse={vz:.1f}mm/s")
            fall_time = self.calculate_fall_time(z, vz)

            if fall_time is not None:
                print(f"Temps de chute estimé: {fall_time:.3f}s")
            
            # Mise à jour des angles virtuels avant de faire le rebond
            self._virtualAngleTheta, self._virtualAnglePhi = self.calculate_angles()
            
            # Décision unique sur la méthode de rebond à utiliser
            rebond_message = None
            
            if z < critical_height and vz < -100:
                # Cas 1: La balle est très basse et descend rapidement
                rebond_message = f"💥 Rebond forcé ! (Hauteur critique: {z:.1f}mm, vitesse: {vz:.1f}mm/s)"
            elif fall_time is not None:
                # Ajustement avec l'offset configuré
                adjusted_fall_time = fall_time - self._bounce_offset
                
                # Cas 2: Détection par temps de chute estimé
                if adjusted_fall_time < bounce_trigger_time:
                    rebond_message = f"💥 Rebond anticipé ! (Impact dans: {fall_time:.3f}s, Ajusté: {adjusted_fall_time:.3f}s)"
            elif z < 150 and vz < -300:
                # Cas 3: Détection de secours par combinaison hauteur/vitesse
                rebond_message = f"💥 Rebond de secours ! (z={z:.1f}mm, vz={vz:.1f}mm/s)"
            
            # Si un rebond a été décidé, l'exécuter
            if rebond_message:
                print(rebond_message)
                self._plate.MakeOneBounce(self._virtualAngleTheta, self._virtualAnglePhi)
                self._bounceAutorised = False
                self._last_bounce_time = current_time
          

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
        Kp = 0.005
        Kd = 0.025  # Gain dérivé vitesse
        Kp=0
 
        # Régulation PD
        theta = - (Kp * ex + Kd * avg_vx) 
        phi   =   (Kp * ey + Kd * avg_vy) 
 
        # Clamp pour éviter les dépassements
        theta = max(-angle_max, min(angle_max, theta))
        phi   = max(-angle_max, min(angle_max, phi)) 
        return theta, phi
    


    