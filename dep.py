import serial
import time

def format_angle(a):
    return f"{a:+03d}"

def send_angles(ser, a1, a2, a3):
    trame = f"{format_angle(a1)}:{format_angle(a2)}:{format_angle(a3)}\n"
    ser.write(trame.encode())
    print(f"Trame envoyée : {trame.strip()}")

    # Attente de confirmation
    while True:
        line = ser.readline().decode().strip()
        if line:
            print(f"ESP32 → {line}")
            if "ANGLES REÇUS" in line:
                break

# --- MAIN ---

# Ouverture du port série
ser = serial.Serial('COM5', 115200, timeout=1)
time.sleep(2)  # Stabilisation

try:
    
    # quatrième envoi
    send_angles(ser, 30, 30, 30)
    time.sleep(0.25)

    # quatrième envoi
    send_angles(ser, 90, 45, 90)
    time.sleep(0.25)

    # quatrième envoi
    send_angles(ser, 60, 60, 60)
    time.sleep(0.25)

    # quatrième envoi
    send_angles(ser, 90, 90, 45)
    time.sleep(0.25)

    # quatrième envoi
    send_angles(ser, 60, 60, 60)
    time.sleep(0.25)
    
    # quatrième envoi
    send_angles(ser, 45, 90, 90)
    time.sleep(0.25)

    # quatrième envoi
    send_angles(ser, 45, 45, 45)
    time.sleep(0.25)

    # quatrième envoi
    send_angles(ser, 60, 60, 60)
    time.sleep(0.25)
    # quatrième envoi
    send_angles(ser, 45, 45, 45)
    time.sleep(0.25)

    # quatrième envoi
    send_angles(ser, 60, 60, 60)
    time.sleep(0.25)

    # quatrième envoi
    send_angles(ser, 45, 45, 45)
    time.sleep(0.25)

    # quatrième envoi
    send_angles(ser, 45, 45, 45)
    time.sleep(0.25)

    # quatrième envoi
    send_angles(ser, 60, 60, 60)
    time.sleep(0.25)
    # quatrième envoi
    send_angles(ser, 45, 45, 45)
    time.sleep(0.25)

    # quatrième envoi
    send_angles(ser, 60, 60, 60)
    time.sleep(0.25)

    # quatrième envoi
    send_angles(ser, 45, 45, 45)
    time.sleep(0.25)


    # quatrième envoi
    send_angles(ser, 60, 60, 60)
    time.sleep(0.1)
    # quatrième envoi
    send_angles(ser, 50, 50, 50)
    time.sleep(0.1)

    # quatrième envoi
    send_angles(ser, 45, 45, 45)
    time.sleep(0.1)

    # quatrième envoi
    send_angles(ser, 50, 50, 50)
    time.sleep(0.1)

    # quatrième envoi
    send_angles(ser, 90, 90, 90)
    time.sleep(0.1)

    # quatrième envoi
    send_angles(ser, 10, 10, 10)
    time.sleep(0.1)

    # quatrième envoi
    send_angles(ser, 90, 10, 10)
    time.sleep(0.1)

    # quatrième envoi
    send_angles(ser, 10, 90, 10)
    time.sleep(0.1)

    # quatrième envoi
    send_angles(ser, 10, 10, 90)
    time.sleep(0.1)

    # quatrième envoi
    send_angles(ser, 10, 90, 10)
    time.sleep(0.1)

    # quatrième envoi
    send_angles(ser, 10, 10, 90)
    time.sleep(0.1)
    # quatrième envoi
    send_angles(ser, 10, 10, 10)
    time.sleep(0.1)

finally:
    ser.close()
