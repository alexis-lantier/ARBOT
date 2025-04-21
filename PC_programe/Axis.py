from Motor import Motor


class Axis:
    def __init__(self):
        self._motor = Motor()
        self._height = 0

    def move(self, height_increment ):
        self._motor.move(height_increment)
        self._height += height_increment
