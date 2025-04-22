from Machine import Machine


class Application:

    def __init__(self):
        self._machine = Machine()

    def Initialisation(self):
        pass

    def Regulation(self):
        pass

    def Afficher_menu(self, A, B, C):
        print("\n=== MENU MANUEL ===")
        print(f"Axe A : {A}")
        print(f"Axe B : {B}")
        print(f"Axe C : {C}")
        print("\nChoisis une action :")
        print("1. + PHI")
        print("2. + THETA")
        print("3. + HEIGHT")
        print("4. - PHI")
        print("5. - THETA")
        print("6. - HEIGHT")
        print("7. Quitter")
        print(">")


    def Test(self):

        while True:

            self.Afficher_menu(
                A=self._machine.GetAnglePhi(),
                B=self._machine.GetAngleTheta(),
                C=self._machine.GetHeight(),
            )
            choix = input("Ton choix : ")

            if choix == "1":
                self._machine.MoveAxisPhi(self._machine.GetAnglePhi() + 1)
            elif choix == "2":
                self._machine.MoveAxisTheta(self._machine.GetAngleTheta() + 1)
            elif choix == "3":
                self._machine.MoveAxisHeigh(self._machine.GetHeight() + 1)
            elif choix == "4":
                self._machine.MoveAxisPhi(self._machine.GetAnglePhi() - 1)
            elif choix == "5":
                self._machine.MoveAxisTheta(self._machine.GetAngleTheta() - 1)
            elif choix == "6":
                self._machine.MoveAxisHeigh(self._machine.GetHeight() - 1)
            elif choix == "7":
                print("Fermeture du menu. À bientôt !")
                break
            else:
                print("Choix invalide, réessaie.")
