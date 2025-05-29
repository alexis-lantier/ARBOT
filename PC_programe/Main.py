from Application import Application
from Cam import Cam
import time 
import serial.tools.list_ports

def main():
    print("=== Démarrage ARBOT ===")
    print("Ports COM disponibles :")
    ports = list(serial.tools.list_ports.comports())
    if not ports:
        print("Aucun port COM détecté. Saisissez manuellement (ex: COM5).")
    else:
        for i, port in enumerate(ports):
            print(f"{i+1}. {port.device} - {port.description}")

    port = input("Entrez le port COM (ex: COM5) ou son numéro : ").strip()
    if ports and port.isdigit():
        idx = int(port) - 1
        if 0 <= idx < len(ports):
            port = ports[idx].device
        else:
            print("Numéro invalide, utilisation de COM5 par défaut.")
            port = "COM5"
    elif not port:
        port = "COM5"  # Valeur par défaut si rien n'est saisi

    baudRate = 115200
    app = Application(port=port, baudrate=baudRate)
    app.Run()
 

if __name__ == "__main__":
    main()
# derniere version