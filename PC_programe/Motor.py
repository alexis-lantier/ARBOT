import numpy as np
from scipy.optimize import brentq
DEBUG = False

class Motor:
    def __init__(self):
        self._angle = -45
        
    def SetAngle(self, hCible):
        
        new_angle = (45/53)* (hCible-53)
        if DEBUG:
            print(f"New angle calculated: {new_angle}")
        # Calculer le mouvement de l'angle
        self._angle = new_angle


