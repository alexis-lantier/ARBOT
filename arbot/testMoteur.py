import time
import serial
import struct

class MotorTester:
    MOTMAGIC = 0x79
    ERROR_FRAME_TYPE = 0x01
    VALID_FRAME_TYPE = 0x00
    MOTOR_FRAME_TYPE = 0x02
    ANGLE_MIN = -120
    ANGLE_MAX = 120
    ERROR_CODES = {
        0x05: "Erreur : dépassement de trame",
        0x04: "Erreur : checksum incorrect",
        0x03: "Erreur : angle invalide",
        0x02: "Erreur : file pleine",
        0x01: "OK",
        0x00: "Aucun code"
    }

    def __init__(self, port='COM6', baudrate=115200):
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
            print("Connexion série établie avec succès.")
        except serial.SerialException as e:
            print(f"Erreur lors de la connexion série : {e}")
            self.ser = None

    def close(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Connexion série fermée.")
        else:
            print("Connexion série déjà fermée ou jamais ouverte.")

    @staticmethod
    def to_uint8(val):      # Convert to uint8 data type
        return val & 0xFF

    def send_angles(self, angle1, angle2, angle3):
        # Vérification des plages d'angles
        for idx, angle in enumerate([angle1, angle2, angle3], start=1):
            if not (self.ANGLE_MIN <= angle <= self.ANGLE_MAX):
                raise ValueError(f"Angle {idx} ({angle}) hors de la plage autorisée [{self.ANGLE_MIN}, {self.ANGLE_MAX}]")
        a1 = self.to_uint8(angle1)
        a2 = self.to_uint8(angle2)
        a3 = self.to_uint8(angle3)
        checksum = self.to_uint8((self.MOTMAGIC + a1 + a2 + a3) % 256)
        frame = struct.pack('>BBBBB', self.MOTMAGIC, a1, a2, a3, checksum)
        self.ser.write(frame)
        self.ser.read(5)
        print(f"Envoyé: {[hex(b) for b in frame]}")

    def read_response(self):
        while True:
            if self.ser.in_waiting >= 4:
                response = self.ser.read(5)
                if len(response) != 4:
                    print("Réponse incomplète reçue.")
                    continue
                motmagic, mode_rec, code_rec, crc = response
                print("Trame reçue :", [hex(b) for b in response])
                if motmagic != self.MOTMAGIC:
                    print("Erreur : mot magique incorrect")
                    continue
                calculated_crc = self.to_uint8((motmagic + mode_rec + code_rec) % 256)
                if crc != calculated_crc:
                    print(f"Erreur CRC : attendu {hex(calculated_crc)}, reçu {hex(crc)}")
                    continue
                msg = self.ERROR_CODES.get(code_rec, f"Code inconnu: {hex(code_rec)}")
                if mode_rec == self.ERROR_FRAME_TYPE:
                    print(f"Erreur : trame d'erreur reçue, {msg}")
                    continue
                elif mode_rec == self.VALID_FRAME_TYPE:
                    print(f"Trame valide reçue, {msg}")
                    break
                elif mode_rec == self.MOTOR_FRAME_TYPE:
                    print(f"Trame moteur reçue, {msg}")
                    break
                else:
                    print(f"Trame inconnue, mode = {hex(mode_rec)}, code = {hex(code_rec)}")
                    continue

    def test_angles(self, angle1, angle2, angle3, N=10):
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        time.sleep(0.1)
        tot_time = 0
        for i in range(N):
            start_time = time.time()
            self.send_angles(angle1, angle2, angle3)
            self.read_response()
            end_time = time.time()
            elapsed_time = (end_time - start_time) * 1000
            tot_time += elapsed_time
            print(f"Temps d'aller-retour pour l'itération {i + 1}: {elapsed_time:.2f} ms")
            time.sleep(0.1)
        avg_time = tot_time / N
        print(f"Temps moyen d'aller-retour sur {N} itérations : {avg_time:.2f} ms")

if __name__ == "__main__":
    try:
        tester = MotorTester(port='COM5', baudrate=115200)
        time.sleep(1)

        # Vide le buffer d'entrée et de sortie
        tester.ser.reset_input_buffer()
        tester.ser.reset_output_buffer()
        time.sleep(0.1)

        tester.send_angles(30, 30, 30)
        tester.send_angles(45, 45, 45)
        tester.send_angles(30, 30, 30)
        tester.send_angles(45, 45, 45)
        tester.send_angles(30, 30, 30)
        tester.send_angles(45, 45, 45)


        tester.close()

    except serial.SerialException as e:
        print(f"Erreur série : {e}")
    except Exception as e:
        print(f"Erreur inattendue : {e}")
    finally:
        if 'tester' in locals():
            tester.close()