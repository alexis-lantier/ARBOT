from Vector import Vector
from Cam import Cam


class Ball:
    def __init__(self, cam):
        self._cam = cam
        self._speed = Vector()
        self._XPosition = 0
        self._YPosition = 0
        self._ZPosition = 0

    def Update(self):
        # Update the ball's position based on its speed
        # appelle de update de la cam
        self._cam.Update()
