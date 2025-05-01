import time
import serial

MOTMAGIC = 0x79          # Motmagic pour le protocole de communication
ERROR_FRAME_TYPE = 0x01  # Type de trame pour les erreurs
VALID_FRAME_TYPE = 0x00  # Type de trame pour confirmation OK

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
        if ser.in_waiting >= 4:
            response = ser.read(4)
            motmagic, mode_rec, code_rec, crc = response
            print("Trame reçue :", [hex(b) for b in response])
            if motmagic != MOTMAGIC:
                print("Erreur : mot magique incorrect")
                continue
            # Vérification du CRC (checksum)
            calculated_crc = to_uint8((motmagic + mode_rec + code_rec) % 256)
            if crc != calculated_crc:
                print(f"Erreur CRC : attendu {hex(calculated_crc)}, reçu {hex(crc)}")
                continue

            if mode_rec == ERROR_FRAME_TYPE:
                print(f"Erreur : trame d'erreur reçue, code erreur = {hex(code_rec)}")
                continue
            elif mode_rec == VALID_FRAME_TYPE:
                print(f"Trame valide reçue, code = {hex(code_rec)}")
                break
            else:
                print(f"Trame inconnue, mode = {hex(mode_rec)}, code = {hex(code_rec)}")
                continue

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

time.sleep(0.05)

# Envoyer des angles de test
try:
    ser.reset_input_buffer()
    ser.reset_output_buffer()
    time.sleep(0.1)

    N = 10  # Nombre d'itérations
    tot_time = 0  # Initialisation du temps total
    
    # mesure du temps moyen d aller retour de la trame avec N mesures
    for i in range(N):
        start_time = time.time()
        send_angles(40, 40, 40)
        read_response()
        end_time = time.time()
        elapsed_time = (end_time - start_time) * 1000  # Convertir en millisecondes
        tot_time += elapsed_time
        print(f"Temps d'aller-retour pour l'itération {i + 1}: {elapsed_time:.2f} ms")
        time.sleep(0.1)
    
    # temps moyen
    avg_time = tot_time / N
    print(f"Temps moyen d'aller-retour sur {N} itérations : {avg_time:.2f} ms")



except serial.SerialException as e:
    print(f"Erreur série : {e}")
except Exception as e:
    print(f"Erreur inattendue : {e}")
