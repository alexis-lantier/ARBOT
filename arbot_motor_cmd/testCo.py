# Python script to test UART communication with ESP32
import serial
import time

# Configure the serial port
SERIAL_PORT = "COM7"  # Replace with your ESP32's COM port
BAUD_RATE = 115200    # Must match the ESP32's UART baud rate
TIMEOUT = 1           # Timeout in seconds
NUM_MEASUREMENTS = 200 # Number of measurements to perform

def test_uart():
    try:
        # Open the serial connection
        with serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT) as ser:
            print(f"Connected to {SERIAL_PORT} at {BAUD_RATE} baud.")

            char_to_send = 'A'
            round_trip_times = []

            # Perform measurements
            for i in range(NUM_MEASUREMENTS):
                start_time = time.time()
                ser.write(char_to_send.encode())  # Send as bytes

                # Wait for the response
                response = ser.read(len(char_to_send))
                end_time = time.time()

                # Check if the response matches
                if response.decode() == char_to_send:
                    round_trip_time = (end_time - start_time) * 1000  # Convert to milliseconds
                    round_trip_times.append(round_trip_time)
                    print(f"Measurement {i + 1}: Round-trip time: {round_trip_time:.2f} ms")
                else:
                    print(f"Measurement {i + 1}: No valid response or mismatch in echoed data.")

            # Calculate and display the average round-trip time
            if round_trip_times:
                average_time = sum(round_trip_times) / len(round_trip_times)
                print(f"\nAverage round-trip time: {average_time:.2f} ms")
            else:
                print("\nNo valid measurements to calculate average.")

    except serial.SerialException as e:
        print(f"Serial error: {e}")
    except Exception as e:
        print(f"Error: {e}")

if __name__ == "__main__":
    test_uart()