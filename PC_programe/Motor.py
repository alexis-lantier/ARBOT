import numpy as np
from scipy.optimize import brentq
DEBUG = False

class Motor:
    def __init__(self):
        self._angle = -45
        
    def SetAngle(self, hCible):
        # hCible = hCible + 50.9
        # if not isinstance(hCible, (int, float)):
        #     raise TypeError(f"hCible must be a real number, got {type(hCible)}")

        # # Coefficients du polynôme
        # coeffs = [-3.393e-5, -0.0003782, 0.6972, 80.07 - hCible]

        # # Convertir les coefficients en float si nécessaire
        # coeffs = [float(c) for c in coeffs]

        # # Calculer les racines
        # racines = np.roots(coeffs)

        # # Filtrer les racines réelles
        # racines_reelles = [r.real for r in racines if np.isreal(r)]

        # print(f"hCible={hCible}, racines={racines_reelles}")

        # new_angle = 0
        # # Choisir la racine dans le domaine physique plausible (ex : -50° à +50°)
        # for r in racines_reelles:
        #     if -50 <= r <= 50:
        #         new_angle = r
        #         break


        new_angle = (45/53)* (hCible-53)
        if DEBUG:
            print(f"New angle calculated: {new_angle}")
        # Calculer le mouvement de l'angle
        self._angle = new_angle


