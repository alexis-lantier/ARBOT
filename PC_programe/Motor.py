import numpy as np


class Motor:
    def __init__(self):
        self._angle = 0
        self._lastAngleMovement = 0

    def SetAgnle(self, hCible):
        coeffs = [-3.393e-5, -0.0003782, 0.6972, 80.07 - hCible]
        racines = np.roots(coeffs)
        racines_reelles = [r.real for r in racines if np.isreal(r)]
        new_angle = 0
        # Choisir celle qui est dans le domaine physique plausible (ex : -50° à +50°)
        for r in racines_reelles:
            if -50 <= r <= 50:
                new_angle = r
                break
        self._lastAngleMovement = new_angle - self._angle
        self._angle = new_angle

    def getAngle(self):
        return self._angle
