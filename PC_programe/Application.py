from Machine import Machine
from threading import Thread, Event
import keyboard
import time

class Application:

    def __init__(self):
        self._machine = Machine()

    def Initialisation(self):
        self._machine._plate.MoveAxisHeigh(50.9)
        
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
        print("\n=== MENU MANUEL ===")
        print("Utilise les touches suivantes pour contrôler :")
        print("z : ^ PHI")
        print("e : > THETA")
        print("r : w HEIGHT")
        print("s : v PHI")
        print("d : < THETA")
        print("f : - HEIGHT")
        print("q : Quitter le menu")

        while True:
            if keyboard.is_pressed('up'):
                self._machine._plate.MoveAxisPhi(self._machine.GetAnglePhi() + 1)
            elif keyboard.is_pressed('right'):
                self._machine._plate.MoveAxisTheta(self._machine.GetAngleTheta() + 30)
            elif keyboard.is_pressed('w'):
                self._machine._plate.MoveAxisHeigh(self._machine.GetHeight() + 1)
            elif keyboard.is_pressed('down'):
                self._machine._plate.MoveAxisPhi(self._machine.GetAnglePhi() - 1)
            elif keyboard.is_pressed('left'):
                self._machine._plate.MoveAxisTheta(self._machine.GetAngleTheta() - 1)
            elif keyboard.is_pressed('s'):
                self._machine._plate.MoveAxisHeigh(self._machine.GetHeight() - 1)
            elif keyboard.is_pressed('q'):
                print("Fermeture du menu. À bientôt !")
                break

            time.sleep(0.1)  # Pour éviter de saturer le CPU et répéter trop vite
