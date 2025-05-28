import time
import serial
import struct

class ConnectionToMicrocontroller:
    # Angles en degrés
    ANGLE_MIN = -120
    ANGLE_MAX = 120

    def __init__(self, port='COM5', baudrate=115200):
        try:
            self.ser = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=1,
                xonxoff=False,
                rtscts=False,
                dsrdtr=False
            )
            self.connected = True
        except serial.SerialException:
            self.ser = None
            self.connected = False

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            return 0  # Succès
        else:
            return 1  # Déjà fermé ou jamais ouvert
    
    @staticmethod
    def format_angle(a):
        return "{:.2f}".format(a)

    def send_angles(self, angle1, angle2, angle3):
        # Vérification des plages
        for idx, angle in enumerate([angle1, angle2, angle3], start=1):
            if not (self.ANGLE_MIN <= angle <= self.ANGLE_MAX):
                return 2  # Code d'erreur pour angle hors plage

        # Conversion float -> string (2 décimales)
        s1 = self.format_angle(angle1)
        s2 = self.format_angle(angle2)
        s3 = self.format_angle(angle3)

        # Trame finale
        frame = f"{s1}:{s2}:{s3}\n"
        print(f"Trame envoyée : {frame.strip()}")

        if self.ser and self.ser.is_open:
            self.ser.write(frame.encode())
            return 0  # Succès
        else:
            return 3  # Port série non ouvert
     
if __name__ == "__main__":
    import time

    try:
        conn = ConnectionToMicrocontroller(port='COM5', baudrate=115200)
        time.sleep(1)

        if conn.ser:
            conn.ser.reset_input_buffer()
            conn.ser.reset_output_buffer()
            time.sleep(0.1)

        conn.send_angles(10, 20, 30)
        response = conn.read_response()
        print(f"Réponse reçue : {response} ({ConnectionToMicrocontroller.ERROR_CODES.get(response, 'Code inconnu')})")

    except serial.SerialException as e:
        print(f"Erreur série : {e}")
    except Exception as e:
        print(f"Erreur inattendue : {e}")
    finally:
        if 'conn' in locals():
            conn.close()