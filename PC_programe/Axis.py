from Motor import Motor


class Axis:
    def __init__(self):
        self._motor = Motor()
        self._height = 0

    def move(self, hCible):
        self._motor.SetAngle(hCible)
        self._height = hCible
