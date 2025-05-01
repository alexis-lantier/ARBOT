import time
import serial

MOTMAGIC = 0x96          # Motmagic pour le protocole de communication
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

time.sleep(0.1)

# Envoyer des angles de test
try:
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    time.sleep(0.1)

    # Commence un chrono
    start_time = time.time()
    send_angles(40, 40, 40)  # Position initiale
    read_response()

    # terminer le chrono
    elapsed_time = time.time() - start_time

    print(f"Temps écoulé pour la première trame : {elapsed_time:.3f} secondes")
    
    """# Boucle de 5 qui bouuge le moteur de 0 à 45° et de 45° à 0°
    for i in range(10):
        send_angles(40, 40, 40)  # Position à 45°
        read_response()
        time.sleep(0.2)
        send_angles(-40, -40, -40)  # Retour à la position initiale
        read_response()
        time.sleep(0.2)"""

except serial.SerialException as e:
    print(f"Erreur série : {e}")
except Exception as e:
    print(f"Erreur inattendue : {e}")
