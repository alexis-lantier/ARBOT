from Connection_Motor import ConnectionMotor
from Axis import Axis
from sympy import symbols, Eq, sin, asin, nsolve




class Plate:
    def __init__(self):
        self._anglePhi = 0
        self._angleTheta = 0
        self._height = 0
        self._ConnectionMotor = ConnectionMotor()
        self._axisA = Axis()
        self._axisB = Axis()
        self._axisC = Axis()

    def MoveAxisHeigh(self, height):
        # monter tous les axes
        AdditionalHeight = height- self._height
        self._axisA.move(self._axisA._height + AdditionalHeight)
        self._axisB.move(self._axisB._height + AdditionalHeight)
        self._axisC.move(self._axisC._height + AdditionalHeight)

        self._ConnectionMotor.SendMMovement(
            self._axisA._motor._lastAngleMovement,
            self._axisB._motor._lastAngleMovement,
            self._axisC._motor._lastAngleMovement,
        )

    def CalculateHeightBasedOnAngle(self,phi_val,theta_val):
        z, H2, d1, d2, phi, theta = symbols('z H2 d1 d2 phi theta')
        offset = 0 # Adjust this value as needed
        d1_val = 0.225 + offset        # j'ai mit la valeur en m
        d2_val = 0.2598 + 2 * offset            
        z_val = self._height

        equation = Eq(z, H2 + d1 * sin(0.5 * asin(d2 / d1 * sin(2 * phi + asin(H2 / d2))) + 0.5 * asin(H2 / d1) + theta) + d2 * sin(2 * phi + asin(H2 / d2)))

        # Substitute numerical values into the equation
        equation_numeric = equation.subs({
            z: z_val,
            d1: d1_val,
            d2: d2_val,
            phi: phi_val,
            theta: theta_val
        })

        # Solve the equation for H2, providing an initial guess
        h2 = nsolve(equation_numeric, H2, 0)  # Initial guess for H2 is 0

        h3 = d1_val * sin(
        0.5 * asin(d2_val / d1_val * sin(2 * phi_val + asin(h2 / d2_val))) +
        0.5 * asin(h2 / d1_val) + theta_val
        )
        h1 = d2_val * sin(2 *phi_val + asin(h2 / d2_val))

        print(f"Calculated heights: h1={h1}, h2={h2}, h3={h3}") # h2 est un nombre complexe ... à corriger

        return h1, h2, h3
    
    
    def MoveAxisPhi(self, angle):
        
        h1,h2,h3 = self.CalculateHeightBasedOnAngle(angle, self._angleTheta)
        self._axisA.move(h1)
        self._axisB.move(h2)
        self._axisC.move(h3)


        self._ConnectionMotor.SendMMovement(
            self._axisA._motor._lastAngleMovement,
            self._axisB._motor._lastAngleMovement,
            self._axisC._motor._lastAngleMovement,
        )



    def MoveAxisTheta(self, angle):

        h1,h2,h3 = self.CalculateHeightBasedOnAngle(self._anglePhi, angle)

        self._axisA.move(h1)
        self._axisB.move(h2)
        self._axisC.move(h3)

        self._ConnectionMotor.SendMMovement(
            self._axisA._motor._lastAngleMovement,
            self._axisB._motor._lastAngleMovement,
            self._axisC._motor._lastAngleMovement,
        )

    def GetAnglePhi(self):
        return self._anglePhi

    def GetAngleTheta(self):
        return self._angleTheta

    def GetHeight(self):
        return self._height
