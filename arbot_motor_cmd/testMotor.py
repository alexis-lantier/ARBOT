import time
import serial

MOTMAGIC = 0xAA  # Motmagic pour le protocole de communication
ERROR_FRAME_TYPE = 0xE1  # Type de trame pour les erreurs
VALID_FRAME_TYPE = 0xA1  # Type de trame pour confirmation OK

# Fonction pour convertir une valeur signée (-128 à 127) en uint8
def to_uint8(val):
    return val & 0xFF

# Fonction pour envoyer 3 angles sur le port série
def send_angles(angle1, angle2, angle3):
    ser.write(MOTMAGIC.to_bytes(1, 'big'))  # Envoi du mot magique
    a1 = to_uint8(angle1)  # Conversion de l'angle 1
    a2 = to_uint8(angle2)  # Conversion de l'angle 2
    a3 = to_uint8(angle3)  # Conversion de l'angle 3
    ser.write(a1.to_bytes(1, 'big'))  # Envoi de l'angle 1
    ser.write(a2.to_bytes(1, 'big'))  # Envoi de l'angle 2
    ser.write(a3.to_bytes(1, 'big'))  # Envoi de l'angle 3
    # Calcul du checksum
    checksum = to_uint8((MOTMAGIC + a1 + a2 + a3) % 256)
    ser.write(checksum.to_bytes(1, 'big'))
    print(f"Envoyé: {[hex(MOTMAGIC), hex(angle1), hex(angle2), hex(angle3), hex(checksum)]}")
    time.sleep(0.1)

# Fonction pour vérifier le checksum
def verify_checksum(frame):
    calculated_checksum = sum(frame[:3]) % 256
    return calculated_checksum == frame[3]

# Lecture de la trame de retour du moteur
def read_response():
    while True:
        if ser.in_waiting >= 1:
            response = ser.read(3)
            print("Trame brute reçue :", [hex(b) for b in response])
            break

# Configuration de la connexion série
try:
    ser = serial.Serial(
        port='COM7',
        baudrate=115200,
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
    print(f"Erreur de connexion série : {e}")
    exit()

time.sleep(2)

# Envoyer des angles de test
try:
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    time.sleep(1)

    send_angles(1, 2, 3)  # Position initiale
    read_response()
except serial.SerialException as e:
    print(f"Erreur série : {e}")
except Exception as e:
    print(f"Erreur inattendue : {e}")
