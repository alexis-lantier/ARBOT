from Machine import Machine


class Application:

    def __init__(self):
        self._machine = Machine()

    def Initialisation(self):
        pass

    def Regulation(self):
        pass

    def Afficher_menu(A, B, C):
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
        
        self.Afficher_menu(x, y, z)
        choix = input("Ton choix : ")

        if choix == "1":
            x += 1
        elif choix == "2":
            x -= 1
        elif choix == "3":
            y += 1
        elif choix == "4":
            y -= 1
        elif choix == "5":
            z += 1
        elif choix == "6":
            z -= 1
        elif choix == "7":
            print("Fermeture du menu. À bientôt !")
            break
        else:
            print("Choix invalide, réessaie.")

# Lancer le menu
if __name__ == "__main__":
    menu_axes()
