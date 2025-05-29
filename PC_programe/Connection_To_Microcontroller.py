import time
import serial
import struct

DEBUG = True

class ConnectionToMicrocontroller:
    # Angles en degrés
    ANGLE_MIN = -120
    ANGLE_MAX = 120

    def __init__(self, port='COM5', baudrate=115200):
        try:
            self.ser = serial.Serial(port, baudrate, timeout=1)
            self.connected = True
        except serial.SerialException:
            self.ser = None
            self.connected = False

    def is_connected(self):
        return self.connected

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            return 0  # Succès
        else:
            return 1  # Déjà fermé ou jamais ouvert
    
    @staticmethod
    def format_angle(a):
        return f"{a:+07.2f}"

    def send_angles(self, a1, a2, a3):
        trame = f"{self.format_angle(a1)}:{self.format_angle(a2)}:{self.format_angle(a3)}\n"
        self.ser.write(trame.encode())
        if DEBUG:
            print(f"Trame envoyée (debug) : {trame.strip()}")

        # Attente de confirmation
        while True:
            line = self.ser.readline().decode().strip()
            
            if line:
                if DEBUG:
                    print(f"ESP32 → {line}")
                if "ANGLES REÇUS" in line:
                    break
     
if __name__ == "__main__":
    conn = ConnectionToMicrocontroller()
    if conn.is_connected():
        print("Connexion réussie.")
        conn.send_angles(-20, -20, -20)
        time.sleep(1)
        conn.send_angles(0, 0, 0)
        time.sleep(1)
        conn.send_angles(20, 20, 20)
        time.sleep(1)
        conn.send_angles(-20, -20, -20)
        conn.close()
    else:
        print("Échec de la connexion au port série.")