from Ball import Ball
from Plate import Plate
from Cam import Cam
import math
import time

#########################################################

class BallPredictor:
    def __init__(self, offsetmot=0.09):
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
        return min(times) if times else None

    def add_prediction(self, height, velocity):
        now = time.time()

        # Fake hauteur pour anticipation moteur
        #fake_offset = min(400, abs(velocity) * 0.03)  # mm
        fake_offset = 1  # mm (expérimentalement à ajuster)
        height_fake = max(1, height - fake_offset)

        t_chute = self.calculate_fall_time(height_fake, velocity)

        if t_chute is not None and 0.05 < t_chute < 1.0:
            t_final = now + t_chute
            self.predictions.append(t_final)

        # Nettoyer les vieilles prédictions (temps déjà dépassés)
        self.predictions = [t for t in self.predictions if t > now]

    def get_activation_time(self):
        if not self.predictions:
            return None
        return sum(self.predictions) / len(self.predictions) - self.offsetmot

    def should_activate_motor(self, current_velocity):
        now = time.time()
        self.predictions = [t for t in self.predictions if t > now]  # nettoyage ici aussi

        if self.activation_done or not self.predictions:
            return False

        t_activation = self.get_activation_time()
        if t_activation is not None and now >= t_activation:
            if current_velocity < 0:
                self.activation_done = True
                return True
        return False

    def reset(self):
        self.predictions.clear()
        self.activation_done = False

##################################################################


class Machine:

    def __init__(self, port="COM5", baudrate=115200):

        self._ball = Ball()
        self._plate = Plate(port=port, baudrate=baudrate)
        self._bounceAutorised = True
        self._last_bounce_time = 0
        self._min_bounce_interval = (
            0.4  # Intervalle minimum entre les rebonds en secondes
        )
        self._bounce_offset = (
            0  # Temps d'avance pour déclencher le rebond avant l'impact
        )

        self._virtualAnglePhi = 0
        self._virtualAngleTheta = 0

        self._predictor = BallPredictor(offsetmot=self._bounce_offset)

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

        current_time = time.time()
        z = self._ball._cam._position.z
        zoffset = 0
        z = z - zoffset

        vz = self._ball._cam._ballSpeed.z
        if abs(vz) < 50:
            vz = 0

        d = self._ball._cam._radius

        if vz < 0:
            self._predictor.add_prediction(z, vz)

            if self._bounceAutorised:
                # Cas 2 : Prédiction multiple
                if self._predictor.should_activate_motor(vz):
                    temps_depuis_dernier_rebond = current_time - self._last_bounce_time
                    if temps_depuis_dernier_rebond < self._min_bounce_interval:
                        return  # Trop tôt pour rebondir à nouveau

                    print(f"💥 Rebond déclenché par prédiction multiple ! {time.time():.3f}")
                    self._plate.MakeOneBounce(
                        self._virtualAngleTheta, self._virtualAnglePhi
                    )
                    self._bounceAutorised = False
                    self._last_bounce_time = current_time
                    self._predictor.reset()
                    return

                # Cas 3 : Diamètre (secours)
                if z < 310:
                    temps_depuis_dernier_rebond = current_time - self._last_bounce_time
                    if temps_depuis_dernier_rebond < self._min_bounce_interval:
                        return  # Trop tôt pour rebondir à nouveau

                    print(f"💥 Rebond déclenché par diamètre ! {time.time():.3f}")
                    self.RegulationCenter()
                    self._plate.MakeOneBounce(
                        self._virtualAngleTheta, self._virtualAnglePhi
                    )
                    self._bounceAutorised = False
                    self._last_bounce_time = current_time
                    self._predictor.reset()
                    return

        # Réautoriser rebond si balle est remontée
        if z > 40 and not self._bounceAutorised:
            self._bounceAutorised = True



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

        # Gains à ajuster selon ton système
        Kp = 0.018
        Kd = 0.015  # Gain dérivé vitesse

        # Régulation PD
        theta = -(Kp * ex + Kd * avg_vx)
        phi = Kp * ey + Kd * avg_vy

        # Clamp pour éviter les dépassements
        theta = max(-angle_max, min(angle_max, theta))
        phi = max(-angle_max, min(angle_max, phi))
        return theta, phi
