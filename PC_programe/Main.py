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
    
    # Affichage des plots après fermeture du programme
    print("\nGénération des graphiques...")
    
    # Création d'une figure avec 3 sous-graphiques
    plt.figure(figsize=(12, 10))
      # Sous-graphique 1: Position Z en fonction du temps
    plt.subplot(3, 1, 1)
    plt.plot(app._machine._ball._cam._timePlot, app._machine._ball._cam._zpositionPlot, 'bo-', label='Position Z', linewidth=2, markersize=4)
    plt.title('Position verticale de la balle en fonction du temps', fontsize=14, fontweight='bold')
    plt.ylabel('Hauteur (mm)', fontsize=12)
    plt.grid(True, alpha=0.3)
    plt.legend(fontsize=10)
    
    # Sous-graphique 2: Vitesse Z en fonction du temps
    plt.subplot(3, 1, 2)
    plt.plot(app._machine._ball._cam._timePlot, app._machine._ball._cam._zspeedPlot, 'ro-', label='Vitesse Z', linewidth=2, markersize=4)
    plt.title('Vitesse verticale de la balle en fonction du temps', fontsize=14, fontweight='bold')
    plt.ylabel('Vitesse (mm/s)', fontsize=12)
    plt.grid(True, alpha=0.3)
    plt.legend(fontsize=10)
    
    # Sous-graphique 3: Accélération Z en fonction du temps
    plt.subplot(3, 1, 3)
    plt.plot(app._machine._ball._cam._timePlot, app._machine._ball._cam._zaccelerationPlot, 'go-', label='Accélération Z', linewidth=2, markersize=4)
    plt.title('Accélération verticale de la balle en fonction du temps', fontsize=14, fontweight='bold')
    plt.xlabel('Index temporel', fontsize=12)
    plt.ylabel('Accélération (mm/s²)', fontsize=12)
    plt.grid(True, alpha=0.3)
    plt.legend(fontsize=10)
    
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
