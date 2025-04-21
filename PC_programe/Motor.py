
class Motor:
    def __init__(self, name, pin1, pin2):
        self._angle = 0

    def setAgnle(self, angle):
        self._angle = angle

    def getAngle(self):
        return self._angle
    