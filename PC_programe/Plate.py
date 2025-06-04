from Connection_To_Microcontroller import ConnectionToMicrocontroller
from Axis import Axis
import time

import numpy as np
from scipy.optimize import fsolve, least_squares

MIN_H = 0
MAX_H = 106.4
DEBUG = False


class Plate:
    def __init__(self,port='COM5', baudrate=115200):
        self._anglePhi = 0
        self._angleTheta = 0
        self._height = 0
        self._connectionToMicrocontroller = ConnectionToMicrocontroller( port=port, baudrate=baudrate)
        self._axisA = Axis()
        self._axisB = Axis()
        self._axisC = Axis()

    def MoveAxisHeigh(self, height):
        old_valueh1 = self._axisA._height
        old_valueh2 = self._axisB._height
        old_valueh3 = self._axisC._height

        # monter tous les axes

        AdditionalHeight = height - self._height

        # Calculer la nouvelle hauteur
        new_valueh1 = self._axisA._height + AdditionalHeight
        new_valueh2 = self._axisB._height + AdditionalHeight
        new_valueh3 = self._axisC._height + AdditionalHeight

        #Vérifier si la nouvelle hauteur est dans les limites
        if not (
            MIN_H <= new_valueh1 <= MAX_H
            and MIN_H <= new_valueh2 <= MAX_H
            and MIN_H <= new_valueh3 <= MAX_H
        ):
            print("La nouvelle hauteur dépasse les limites autorisées.")
            return
        

        self._axisA.move(new_valueh1)
        self._axisB.move(new_valueh2)
        self._axisC.move(new_valueh3)
        # Update the height
        self._height = height
        self._connectionToMicrocontroller.send_angles(
            self._axisA._motor._angle,
            self._axisB._motor._angle,
            self._axisC._motor._angle,
        )
        if DEBUG:
            print(
                f"Axis A height: {self._axisA._height}, Axis B height: {self._axisB._height}, Axis C height: {self._axisC._height}"
            )

    def CalculateHeightBasedOnAngle(self, phi_val, theta_val):
        MIN_H_FOR_ANGLE = 0
        MAX_H_FOR_ANGLE = 105.4
        d1_val = 201
        d2_val = 232
        z_val = self._height  # Assure-toi que z_val est bien entre 50.9 et 107.4

        phi_val = np.radians(phi_val)
        theta_val = np.radians(theta_val)

        def equations(vars):
            h1, h2, h3 = vars

            if any(h < MIN_H_FOR_ANGLE or h > MAX_H_FOR_ANGLE for h in [h1, h2, h3]):
                return [1e6, 1e6, 1e6]

            eq1 = (
                0.5 * np.arcsin(h1 / d2_val)
                - 0.5 * np.arcsin(h2 / d2_val)
                - phi_val
            )
            eq2 = (
                -0.5 * np.arcsin(h1 / d1_val)
                - 0.5 * np.arcsin(h2 / d1_val)
                + np.arcsin(h3 / d1_val)
                - theta_val
            )
            eq3 = (h1 + h2 + h3) / 3 - z_val
            return [eq1, eq2, eq3]

        guesses = [
            [50, 50, 50],  
            [z_val, z_val, z_val],
            [MAX_H_FOR_ANGLE, MIN_H_FOR_ANGLE, z_val],
            [MIN_H_FOR_ANGLE, MAX_H_FOR_ANGLE, z_val],
        ]

       

        lower_bounds = [MIN_H, MIN_H, MIN_H]
        upper_bounds = [MAX_H, MAX_H, MAX_H]
        for guess in guesses:
            result = least_squares(
                equations,
                guess,
                bounds=(lower_bounds, upper_bounds),
                xtol=1e-3,
                max_nfev=100000
            )
            if result.success:
                h1, h2, h3 = result.x
                if DEBUG : 
                    print(f"Solution trouvée : h1={h1:.2f}, h2={h2:.2f}, h3={h3:.2f}")
                return h1, h2, h3  # <- RETURN ici

        print("Pas de solution trouvée dans les bornes.")
        return None  # <- RETURN ici si rien trouvé




    def MoveAxisPhi(self, angle):

        h1, h2, h3 = self.CalculateHeightBasedOnAngle(angle, self._angleTheta)

        # Check if the calculated heights are within the limits
        if not (MIN_H <= h1 <= MAX_H and MIN_H <= h2 <= MAX_H and MIN_H <= h3 <= MAX_H):
            print("La nouvelle hauteur dépasse les limites autorisées.")
            return
        
        self._axisA.move(h1)
        self._axisB.move(h2)
        self._axisC.move(h3)

        # Update the angles
        self._anglePhi = angle
        self._height = (h1 + h2 + h3) / 3

        self._connectionToMicrocontroller.send_angles(
            self._axisA._motor._angle,
            self._axisB._motor._angle,
            self._axisC._motor._angle,
        )

    def MoveAxisTheta(self, angle):

        h1, h2, h3 = self.CalculateHeightBasedOnAngle(self._anglePhi, angle)
        #Check if the calculated heights are within the limits
        if not (MIN_H <= h1 <= MAX_H and MIN_H <= h2 <= MAX_H and MIN_H <= h3 <= MAX_H):
            print("La nouvelle hauteur dépasse les limites autorisées.")
            return
        
        self._axisA.move(h1)
        self._axisB.move(h2)
        self._axisC.move(h3)

        # Update the angles
        self._angleTheta = angle
        self._height = (h1 + h2 + h3) / 3

        self._connectionToMicrocontroller.send_angles(
            self._axisA._motor._angle,
            self._axisB._motor._angle,
            self._axisC._motor._angle,
        )

    def MakeOneBounce(self,virtualAngleTheta, virtualAnglePhi):

        
        bounce_height = 30
    
        self.MoveAxisPhi(virtualAnglePhi)
        self.MoveAxisTheta(virtualAngleTheta)

        self._axisA.move(self._axisA._height + bounce_height )
        self._axisB.move(self._axisB._height + bounce_height)
        self._axisC.move(self._axisC._height + bounce_height)
        self._connectionToMicrocontroller.send_angles(
            self._axisA._motor._angle,
            self._axisB._motor._angle,
            self._axisC._motor._angle,
        )
        time.sleep(0.1)  # Attendre un peu pour le rebond

        # redescendre les axes a la position initiale
        self._axisA.move(self._axisA._height - bounce_height )
        self._axisB.move(self._axisB._height - bounce_height )
        self._axisC.move(self._axisC._height - bounce_height )
        self._connectionToMicrocontroller.send_angles(
            self._axisA._motor._angle,
            self._axisB._motor._angle,
            self._axisC._motor._angle,
        )
        print("💥 Rebond déclenché !")

    def GetAnglePhi(self):
        return self._anglePhi

    def GetAngleTheta(self):
        return self._angleTheta

    def GetHeight(self):
        return self._height
