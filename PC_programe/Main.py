from Application import Application
from Cam import Cam
import time 
import serial.tools.list_ports
from matplotlib import pyplot as plt

DEBUG = False

def main():
    print("=== Démarrage ARBOT ===")
    print("Ports COM disponibles :")
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("Aucun port COM détecté. Saisissez manuellement (eqx: 5).")
    else:
        for i, port in enumerate(ports):
            print(f"{i+1}. {port.device} - {port.description}")

    port = input("Entrez le port COM (ex: COM5 ou 13 pour COM13) ou son numéro dans la liste : ").strip()
    if ports and port.isdigit():
        idx = int(port) - 1
        if 0 <= idx < len(ports):
            port = ports[idx].device
        else:
            # Si l'utilisateur tape un nombre, on tente COM{nombre}
            port = f"COM{port}"
    elif port.isdigit():
        port = f"COM{port}"
    elif not port:
        port = "COM5"  # Valeur par défaut si rien n'est saisi

    baudRate = 115200
    app = Application(port=port, baudrate=baudRate)
    if DEBUG:
        app.Test()
    else:
        app.Run()
    plt.subplot(2, 1, 1)
    plt.plot(app._machine._ball._cam._timePlot, app._machine._ball._cam._zpositionPlot, 'b-', label='Position Z')
    plt.title('Hauteur de la balle en fonction du temps')
    plt.ylabel('Hauteur (mm)')
    plt.grid(True)
    plt.legend()
    
    # Sous-graphique 2: Vitesse Z en fonction du temps
    plt.subplot(2, 1, 2)
    plt.plot(app._machine._ball._cam._timePlot, app._machine._ball._cam._zspeedPlot, 'r-', label='Vitesse Z')
    plt.title('Vitesse verticale en fonction du temps')
    plt.xlabel('Index temporel')
    plt.ylabel('Vitesse (mm/s)')
    plt.grid(True)
    plt.legend()
    
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
