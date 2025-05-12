from Connection_Cam import ConnectionCam
from Vector import Vector
import cv2
import numpy as np
import time
from collections import deque

WIDTH = 640
HEIGHT = 480
camera_index = 1


class Cam:
    def __init__(self, connectionCam):
        self._connectionCam = connectionCam  # pas sur d'avoir besoin
        self._previous_position = Vector(None, None, None)
        self._position = Vector()
        self._ballSpeed = Vector()
        self._previous_time = None
        cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
        if not cap.isOpened():
            print("Erreur: Impossible d'ouvrir la caméra.")
            exit()
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
        self._cap = cap
        self._lower_orange = np.array([94, 70, 56, 39, 30])
        self._upper_orange = np.array([25, 255, 255])

        # calibration hauteur
        diam_px = np.array([94, 70, 56, 39, 30])  # Diamètre en pixels
        haut = np.array([0, 10, 50, 100, 200])  # Hauteur réelle correspondante en mm
        coefficients = np.polyfit(diam_px, haut, 2)
        self._poly = np.poly1d(coefficients)

    def detect_ball(frame, lower_orange, upper_orange):
        """Applique un masque HSV et détecte la balle, retourne le centre et le rayon"""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, lower_orange, upper_orange)
        mask = cv2.erode(mask, None, iterations=1)
        mask = cv2.dilate(mask, None, iterations=1)

        contours, _ = cv2.findContours(
            mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            ((x, y), radius) = cv2.minEnclosingCircle(largest_contour)
            if radius > 5:  # Filtrage des valeurs aberrantes
                return (int(x), int(y)), radius
        return None, None

    def calculate_position(center, radius, poly):
        """Calcule la position X, Y, et Z à partir de la détection de la balle"""
        if center and radius:
            x, y = center
            z = poly(2 * radius)  # Conversion diamètre -> hauteur en mm
            return x, y, z
        return None, None, None

    def calculate_speed(prev_x, prev_y, prev_z, prev_time, x, y, z):
        """Calcule la vitesse de la balle en X, Y et Z"""
        vitesse_x, vitesse_y, vitesse_z = 0, 0, 0
        current_time = time.time()

        if x is not None and prev_x is not None and prev_time is not None:
            delta_t = current_time - prev_time
            if delta_t > 0:
                vitesse_x = (x - prev_x) / delta_t  # px/s
                vitesse_y = (y - prev_y) / delta_t  # px/s
                vitesse_z = (z - prev_z) / delta_t if z is not None else 0  # mm/s

        return vitesse_x, vitesse_y, vitesse_z, current_time

    def GetSpeed(self):
        VectorSpeed = self._ballSpeed
        return VectorSpeed

    def GetPosition(self):
        VectorPosition = self._position
        return VectorPosition

    def Update(self):
        frame = cv2.resize(frame, (640, 480))
        center, radius = self.detect_ball(frame, self._lower_orange, self._upper_orange)
        x, y, z = self.calculate_position(center, radius, self._poly)
        self._position = Vector(x, y, z)
        vitesse_x, vitesse_y, vitesse_z, self._previous_time = self.calculate_speed(
            self._previous_position.x,
            self._previous_position.y,
            self._previous_position.z,
            self._previous_time,
            self._position.x,
            self._position.y,
            self._position.z,
        )
        self._previous_position = self._position

    # utilisation de la connection pour mettre à jour la vitesse de la balle
