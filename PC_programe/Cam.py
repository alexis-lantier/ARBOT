from Vector import Vector
import cv2
import numpy as np
import time
from collections import deque

WIDTH = 640
HEIGHT = 480
CAMERA_INDEX = 0

#### début des modifs
class Cam:
    def __init__(self):
        """Initialise la caméra et les paramètres nécessaires."""
        self._previous_position = Vector(None, None, None)
        self._position = Vector()
        self._ballSpeed = Vector()
        self._previous_time = None
        self._radius = None

        # Initialisation de la caméra
        self._cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
        if not self._cap.isOpened():
            print(f"Erreur: Impossible d'ouvrir la caméra avec l'index {CAMERA_INDEX}.")
            exit()

        self._cap.set(3, WIDTH)
        self._cap.set(4, HEIGHT)

        # Plage de couleurs pour détecter la balle orange
        self._lower_orange = np.array([5, 150, 150])
        self._upper_orange = np.array([25, 255, 255])

       

    def get_height(self):
        a= -0.0004
        b= 0.1338
        c= -17.822
        d= 852.27
        x= self._radius*2
        return a*x**3 + b*x**2 + c*x + d
           

   
    
    def detect_ball(self, frame):
        mask = cv2.inRange(cv2.cvtColor(frame, cv2.COLOR_BGR2HSV), self._lower_orange, self._upper_orange)
        mask = cv2.dilate(cv2.erode(mask, None, 1), None, 1)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            ((x, y), r) = cv2.minEnclosingCircle(max(contours, key=cv2.contourArea))
            if r > 5:
                return (int(x), int(y)),r  # Retourne centre et diamètre
        return None, None

    
    

    def calculate_position(self, center, radius):
        """Calcule la position X, Y, et Z à partir de la détection de la balle."""
        if center and radius:
            x, y = center
            z = self.get_height()  # Conversion diamètre -> hauteur en mm
            _position = Vector(x, y, z)
            return x, y, z
        return None, None, None

    def calculate_speed(self, prev_x, prev_y, prev_z, prev_time, x, y, z):
        """Calcule la vitesse de la balle en X, Y et Z."""
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
        """Retourne la vitesse actuelle de la balle."""
        return self._ballSpeed

    def GetPosition(self):
        """Retourne la position actuelle de la balle."""
        return self._position

    def Update(self):
        """Met à jour la position et la vitesse de la balle."""
        ret, frame = self._cap.read()
        if not ret:
            print("Erreur: Impossible de lire la vidéo.")
            return

        frame = cv2.resize(frame, (WIDTH, HEIGHT))
        center, self._radius = self.detect_ball(frame)
        x, y, z = self.calculate_position(center, self._radius)
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
        self._ballSpeed = Vector(vitesse_x, vitesse_y, vitesse_z)
        self._previous_position = self._position

    def display(self):
        """Affiche la vidéo avec les informations de suivi de la balle."""
        ret, frame = self._cap.read()
        if not ret or frame is None:
            print("Erreur: Impossible de lire la vidéo.")
            return

        center = (int(self._position.x), int(self._position.y)) if self._position.x and self._position.y else None
        vitesse_x, vitesse_y, vitesse_z = self._ballSpeed.x, self._ballSpeed.y, self._ballSpeed.z
        x, y, z = self._position.x, self._position.y, self._position.z
        pts = deque(maxlen=16)

        if center and self._radius is not None:
            pts.appendleft(center)
            cv2.circle(frame, center, int(self._radius), (0, 255, 255), 2)
            cv2.circle(frame, center, 3, (0, 0, 255), -1)

        for i in range(1, len(pts)):
            if pts[i - 1] is None or pts[i] is None:
                continue
            thickness = int(np.sqrt(16 / float(i + 1)) * 1.5)
            cv2.line(frame, pts[i - 1], pts[i], (0, 255, 0), thickness)

        # Affichage des informations
        text_position = f"X: {x if x else '-'} | Y: {y if y else '-'} | Z: {z:.2f} mm" if z else "Calcul Z en attente"
        text_vitesse = f"Vx: {vitesse_x:.2f} px/s | Vy: {vitesse_y:.2f} px/s | Vz: {vitesse_z:.2f} mm/s"

        cv2.putText(frame, text_position, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        cv2.putText(frame, text_vitesse, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)

        # Affichage final
        cv2.imshow("Tracking Balle ORANGE avec Vitesse", frame)

        # Quitter avec la touche 'q'
        if cv2.waitKey(1) & 0xFF == ord("q"):
            self._cap.release()
            cv2.destroyAllWindows()
            return

    def release(self):
        """Libère les ressources de la caméra."""
        self._cap.release()
        cv2.destroyAllWindows()

    def display_loop(self, stop_event):
        while not stop_event.is_set():
            self.display()