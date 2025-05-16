from Machine import Machine
from threading import Thread, Event
import keyboard

class Application:

    def __init__(self):
        self._machine = Machine()

    def Initialisation(self):
        pass

    def Regulation(self):
        self._machine._ball.Update()
        self._machine.RegulationCenter()
        self._machine.RegulationBounce()

    def Run(self):

        stop_event = Event()
        update_thread = Thread(
            target=self._machine._ball._cam.display_loop,  
            args=(stop_event,),
            daemon=True
        )
        update_thread.start()
        self.Initialisation()
        while True:
            self.Regulation()


            # print("pos X :",self._machine._ball._XPosition)
            # print("pos Y :",self._machine._ball._YPosition)
            # print("pos Z :",self._machine._ball._ZPosition)
            if keyboard.is_pressed('q'):
                print("Arrêt demandé par l'utilisateur.")
                stop_event.set()
                break
        update_thread.join()

        self._machine._ball._cam.release()
        
            
        

    def Afficher_menu(self, A, B, C):
        print("\n=== MENU MANUEL ===")
        print(f"Axe PHI : {A}")
        print(f"Axe TETA : {B}")
        print(f"Axe Z : {C}")
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
                self._machine._plate.MoveAxisPhi(self._machine.GetAnglePhi() + 1)
            elif choix == "2":
                self._machine._plate.MoveAxisTheta(self._machine.GetAngleTheta() + 1)
            elif choix == "3":
                self._machine._plate.MoveAxisHeigh(self._machine.GetHeight() + 1)
            elif choix == "4":
                self._machine._plate.MoveAxisPhi(self._machine.GetAnglePhi() - 1)
            elif choix == "5":
                self._machine._plate.MoveAxisTheta(self._machine.GetAngleTheta() - 1)
            elif choix == "6":
                self._machine._plate.MoveAxisHeigh(self._machine.GetHeight() - 1)
            elif choix == "7":
                print("Fermeture du menu. À bientôt !")
                break
            else:
                print("Choix invalide, réessaie.")
