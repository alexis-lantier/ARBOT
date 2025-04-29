import numpy as np


class Motor:
    def __init__(self):
        self._angle = -45
        
    def SetAngle(self, hCible):
        if not isinstance(hCible, (int, float)):
            raise TypeError(f"hCible must be a real number, got {type(hCible)}")

        # Coefficients du polynôme
        coeffs = [-3.393e-5, -0.0003782, 0.6972, 80.07 - hCible]

        # Convertir les coefficients en float si nécessaire
        coeffs = [float(c) for c in coeffs]

        # Calculer les racines
        racines = np.roots(coeffs)

        # Filtrer les racines réelles
        racines_reelles = [r.real for r in racines if np.isreal(r)]

        new_angle = 0
        # Choisir la racine dans le domaine physique plausible (ex : -50° à +50°)
        for r in racines_reelles:
            if -50 <= r <= 50:
                new_angle = r
                break

        # Calculer le mouvement de l'angle
        self._angle = new_angle


    def getAngle(self):
        return self._angle
