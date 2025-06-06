from Ball import Ball
from Plate import Plate
from Cam import Cam
import math
import time

#########################################################
def estimate_time_to_hit(z, vz, g=9810):
    if vz >= 0:
        return None
    a = 0.5 * g
    b = -vz
    c = -z
    delta = b**2 - 4*a*c
    if delta < 0:
        return None
    t1 = (-b + math.sqrt(delta)) / (2*a)
    t2 = (-b - math.sqrt(delta)) / (2*a)
    positive_times = [t for t in (t1, t2) if t > 0]
    if not positive_times:
        return None
    
    # COMPENSER LE DÉLAI MÉCANIQUE
    raw_time = min(positive_times)
    mechanical_delay = 0.15  # 150ms - À AJUSTER
    return max(0.001, raw_time - mechanical_delay)


##################################################################


class Machine:

    def __init__(self, port="COM5", baudrate=115200):

        self._ball = Ball()
        self._plate = Plate(port=port, baudrate=baudrate)
        self._bounceAutorised = True
        self._last_bounce_time = 0
        self._virtualAnglePhi = 0
        self._virtualAngleTheta = 0

    

    def GetAnglePhi(self):
        # Get the angle phi from the Ball object
        return self._plate.GetAnglePhi()

    def GetAngleTheta(self):

        return self._plate.GetAngleTheta()

    def GetHeight(self):

        return self._plate.GetHeight()

    def RegulationCenter(self):
        self._virtualAngleTheta, self._virtualAnglePhi = self.calculate_angles()

    def RegulationBounce(self):
        if self._ball._cam._radius is None:
            return
        
        z = self._ball._cam._position.z
        vz = self._ball._cam._ballSpeed.z

        min_bounce_interval = 0.3  # secondes, ajustable

        anticipation_time = 0.20  # 200 ms anticipation
        # Position anticipée de la balle
        z_future = z + vz * anticipation_time

        if vz < 0:
            t_to_hit = estimate_time_to_hit(z_future, vz)
            current_time = time.time()
            # UTILISER LE MÉCANISME DE REBOND AUTORISÉ
            if t_to_hit and t_to_hit < 0.30 and self._bounceAutorised:
                if current_time - self._last_bounce_time > min_bounce_interval:
                    print(f"💥 Rebond déclenché à {current_time:.3f}, t_to_hit = {t_to_hit:.3f}")
                    self._plate.MakeOneBounce(self._virtualAngleTheta, self._virtualAnglePhi)
                    self._last_bounce_time = current_time
                    self._bounceAutorised = False  # Désactiver après rebond
            
            if t_to_hit is not None:
                print(f"[DEBUG] t_to_hit = {t_to_hit:.4f}s | z = {z:.1f} mm | vz = {vz:.1f} mm/s")
        
        # Rebond d'urgence si balle très basse
        if z < 40 and vz < -100:
            if current_time - self._last_bounce_time > min_bounce_interval:
                print(f"🚨 REBOND D'URGENCE à z={z:.1f}mm, vz={vz:.1f}mm/s")
                self._plate.MakeOneBounce(self._virtualAngleTheta, self._virtualAnglePhi)
                self._last_bounce_time = current_time
                return

        # Réactiver quand la balle est haute
        if z > 80 and not self._bounceAutorised:
            self._bounceAutorised = True
            print(f"✅ Rebond réautorisé à z={z:.1f}mm")

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
            self._vx_history = [0.0] * 5
            self._vy_history = [0.0] * 5
            self._history_index = 0

        # Normalisation de z
        z_normalisé = max(0.0, min(1.0, 1 - z / z_max)) + 0.3

        # Mise à jour de l'historique des vitesses
        self._vx_history[self._history_index] = vx
        self._vy_history[self._history_index] = vy
        self._history_index = (self._history_index + 1) % 5

        # Moyenne glissante
        avg_vx = sum(self._vx_history) / 5
        avg_vy = sum(self._vy_history) / 5

        if 1:
            avg_vx = vx
            avg_vy = vy

        gain_global =0.3
        # Gains à ajuster selon ton système
        Kp = 0.03
        Kd = 0.023  # Gain dérivé vitesse
        Kd = Kd * gain_global
        Kp = Kp * gain_global


        # Régulation PD
        theta = -(Kp * ex + Kd * avg_vx)
        phi = Kp * ey + Kd * avg_vy

        # Clamp pour éviter les dépassements
        theta = max(-angle_max, min(angle_max, theta))
        phi = max(-angle_max, min(angle_max, phi))
        return theta, phi
