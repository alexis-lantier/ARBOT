from Connection_Cam import ConnectionCam
from Vector import Vector


class Cam:
    def __init__(self, connectionCam):
        self._connectionCam = connectionCam
        self._ballSpeed = Vector()

    def GetSpeed(self):
        VectorSpeed = self._ballSpeed
        return VectorSpeed

    def Update(self):
        pass

    # utilisation de la connection pour mettre à jour la vitesse de la balle
