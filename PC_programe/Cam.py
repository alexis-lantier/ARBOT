from Vector import Vector
import cv2
import numpy as np
import time
from collections import deque

WIDTH = 640
HEIGHT = 480
CAMERA_INDEX = 1


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
        self._cap.set(cv2.CAP_PROP_FPS, 120)
        if not self._cap.isOpened():
            print(f"Erreur: Impossible d'ouvrir la caméra avec l'index {CAMERA_INDEX}.")
            exit()

        self._cap.set(3, WIDTH)
        self._cap.set(4, HEIGHT)

        # Plage de couleurs pour détecter la balle orange
        self._lower_orange = np.array([5, 80, 80])
        self._upper_orange = np.array([25, 255, 255])

        # Historiques pour lisser les vitesses
        self._vx_history = deque([0] * 4, maxlen=4)
        self._vy_history = deque([0] * 4, maxlen=4)
        self._vz_history = deque([0] * 4, maxlen=4)

        self._cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)  # Désactive l'autofocus
        time.sleep(0.1)
        self._cap.set(cv2.CAP_PROP_FOCUS, 300)
        time.sleep(0.1)
        self._cap.set(cv2.CAP_PROP_FOCUS, 350)
        # self._cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        self._cap.set(cv2.CAP_PROP_EXPOSURE, -4)
        self._cap.set(cv2.CAP_PROP_EXPOSURE, -6)
        self._cap.set(cv2.CAP_PROP_BRIGHTNESS, 300)  ##plot
        self._timeIndex = 0
        self._zpositionPlot = [0]
        self._zspeedPlot = [0]
        self._zaccelerationPlot = [0]
        self._timePlot = [0]
        self._previous_vitesse_z = 0
        self._previous_speed_time = None

    def get_height(self):
        a = -0.0004
        b = 0.1338
        c = -17.822
        d = 852.27
        x = self._radius * 2
        return a * x**3 + b * x**2 + c * x + d

    def detect_ball(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Masque plus doux
        mask = cv2.inRange(hsv, self._lower_orange, self._upper_orange)
        mask = cv2.GaussianBlur(mask, (5, 5), 0)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            best_contour = max(contours, key=cv2.contourArea)

            # Option 1 : Diamètre du cercle englobant
            ((x, y), r) = cv2.minEnclosingCircle(best_contour)

            # Option 2 : Diamètre estimé par ellipse (plus précis si la balle n’est pas parfaitement ronde)
            if len(best_contour) >= 5:
                ellipse = cv2.fitEllipse(best_contour)
                major_axis = max(ellipse[1])  # hauteur ou largeur
                return (int(x), int(y)), major_axis / 2  # rayon + précis

            if r > 5:
                return (int(x), int(y)), r  # Rayon brut

        return None, None

    def calculate_position(self, center, radius):
        """Calcule la position X, Y, et Z à partir de la détection de la balle."""
        if center and radius:
            x, y = center
            z = self.get_height()  # Conversion diamètre -> hauteur en mm
            _position = Vector(x, y, z)
            x = x - WIDTH / 2
            y = y - HEIGHT / 2
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
        """Met à jour la position et la vitesse de la balle avec lissage."""
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

        # Ajoute les vitesses aux historiques
        self._vx_history.append(vitesse_x)
        self._vy_history.append(vitesse_y)
        self._vz_history.append(vitesse_z)

        # Calcule les vitesses moyennes (lissage)
        avg_vx = sum(self._vx_history) / len(self._vx_history)
        avg_vy = sum(self._vy_history) / len(self._vy_history)
        avg_vz = sum(self._vz_history) / len(
            self._vz_history
        )  # Mets à jour la vitesse lissée
        self._ballSpeed = Vector(avg_vx, avg_vy, avg_vz)

        # Calcul de l'accélération Z
        acceleration_z = 0
        current_time = time.time()
        if self._previous_speed_time is not None:
            delta_t = current_time - self._previous_speed_time
            if delta_t > 0:
                acceleration_z = (avg_vz - self._previous_vitesse_z) / delta_t  # mm/s²

        self._previous_vitesse_z = avg_vz
        self._previous_speed_time = current_time
        self._previous_position = self._position

        if self._position.GetZvalue() is not None:
            if self._position.GetZvalue() > 350:
                self._radius = None  # Réinitialise le rayon si la balle est trop haute

        # Mise à jour des plots
        self._timePlot.append(self._timeIndex)
        self._timeIndex += 1
        self._zpositionPlot.append(z)
        self._zspeedPlot.append(vitesse_z)
        self._zaccelerationPlot.append(acceleration_z)

    def display(self):
        """Affiche la vidéo avec les informations de suivi de la balle."""
        ret, frame = self._cap.read()
        if not ret or frame is None:
            print("Erreur: Impossible de lire la vidéo.")
            return

        center = (
            (int(self._position.x + WIDTH / 2), int(self._position.y + HEIGHT / 2))
            if self._position.x and self._position.y
            else None
        )
        vitesse_x, vitesse_y, vitesse_z = (
            self._ballSpeed.x,
            self._ballSpeed.y,
            self._ballSpeed.z,
        )
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
        text_position = (
            f"X: {x if x else '-'} | Y: {y if y else '-'} | Z: {z:.2f} mm"
            if z
            else "Calcul Z en attente"
        )
        text_vitesse = f"Vx: {vitesse_x:.2f} px/s | Vy: {vitesse_y:.2f} px/s | Vz: {vitesse_z:.2f} mm/s"

        cv2.putText(
            frame,
            text_position,
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
        )
        cv2.putText(
            frame,
            text_vitesse,
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2,
        )

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

            # v2
