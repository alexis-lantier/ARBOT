from Vector import Vector
from Cam import Cam


class Ball:
    def __init__(self):
        self._cam = Cam()
        self._speed = Vector()
        self._XPosition = 0
        self._YPosition = 0
        self._ZPosition = 0

    def Update(self):
    
        self._cam.Update()
        self._XPosition = self._cam.GetPosition().x
        self._YPosition = self._cam.GetPosition().y
        self._ZPosition = self._cam.GetPosition().z
        self._speed = self._cam.GetSpeed()

    def update_loop(self, stop_event):
        while not stop_event.is_set():
            self.Update()
            stop_event.wait(0.01)

