import numpy as np


class Motor:
    def __init__(self, name, pin1, pin2):
        self._angle = 0

    def SetAgnle(self, Height_increment):
        # incérment grace à la dérivée ( première idée peut etre fausse)
        AngleIncrement = self.FindAngle_increment(Height_increment)

    def FindAngleIncrement(
        self, Height_increment
    ):  # j'ai utilisé le calcul numérique de l'inverse pour retrouver la valeur de l'angle
        # Coefficients de l'équation cubique : a*α^3 + b*α^2 + c*α + d = h
        # On résout : a*α^3 + b*α^2 + c*α + d - h_cible = 0
        a = -3.393e-5
        b = -0.0003782
        c = 0.6972
        d = 80.07 - (self.Find_Height() + Height_increment)
        coeffs = [a, b, c, d]

        racines = np.roots(coeffs)

        racines_reelles = [r.real for r in racines if np.isreal(r)]
        new_angle = 0

        # Choisir celle qui est dans le domaine physique plausible (ex : -50° à +50°)
        for r in racines_reelles:
            if -50 <= r <= 50:
                return r - self.angle

        return self.angle  # Si aucune racine valable

    def getAngle(self):
        return self._angle

    def Find_Height(self):  # equation inverse de la hauteur
        return (
            -3.393e-5 * self.angle**3
            - 0.0003782 * self.angle**2
            + 0.6972 * self.angle
            + 80.07
        )
