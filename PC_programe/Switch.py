from Connection_To_Microcontroller import ConnectionToMicrocontroller


class Switch:
    def __init__(self, name, state=False):
        self.state = state

    def GetState(self):
        return self.state

    def SetState(self, state):
        # Set the state of the switch avec le microcontroleur
        pass
