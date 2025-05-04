import time
import serial
import struct

class ConnectionToMicrocontroller:

    MOTMAGIC = 0x79
    ERROR_FRAME_TYPE = 0x01
    VALID_FRAME_TYPE = 0x00
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

    def __init__(self, port='COM7', baudrate=115200):
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
    def to_uint8(val):
        return val & 0xFF

    def send_angles(self, angle1, angle2, angle3):
        for idx, angle in enumerate([angle1, angle2, angle3], start=1):
            if not (self.ANGLE_MIN <= angle <= self.ANGLE_MAX):
                return 2  # Code d'erreur pour angle hors plage
        a1 = self.to_uint8(angle1)
        a2 = self.to_uint8(angle2)
        a3 = self.to_uint8(angle3)
        checksum = self.to_uint8((self.MOTMAGIC + a1 + a2 + a3) % 256)
        frame = struct.pack('>BBBBB', self.MOTMAGIC, a1, a2, a3, checksum)
        if self.ser and self.ser.is_open:
            self.ser.write(frame)
            return 0  # Succès
        else:
            return 3  # Port série non ouvert

    def read_response(self):
        start_time = time.time()
        while True:
            if self.ser and self.ser.in_waiting >= 4:
                response = self.ser.read(4)
                if len(response) != 4:
                    return 4  # Réponse incomplète
                motmagic, mode_rec, code_rec, crc = response
                if motmagic != self.MOTMAGIC:
                    return 5  # Mot magique incorrect
                calculated_crc = self.to_uint8((motmagic + mode_rec + code_rec) % 256)
                if crc != calculated_crc:
                    return 6  # Erreur CRC
                if mode_rec == self.ERROR_FRAME_TYPE:
                    return code_rec  # Retourne le code d'erreur reçu
                elif mode_rec == self.VALID_FRAME_TYPE:
                    return code_rec  # Retourne le code OK reçu
                else:
                    return 7  # Trame inconnue
            if time.time() - start_time > 2:
                return 8  # Timeout    