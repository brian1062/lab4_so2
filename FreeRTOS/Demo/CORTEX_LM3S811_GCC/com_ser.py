import serial
import time
import argparse

def main():
    # Set the command line argument
    parser = argparse.ArgumentParser(description="Send messages via UART to a specific device.")
    parser.add_argument("pts", type=int, help="pts port number (for example, 2 for /dev/pts/2)")
    args = parser.parse_args()

    # Configure the serial port
    port = f'/dev/pts/{args.pts}'
    baud_rate = 9600  # Adjust the baud rate as needed

    try:
        # Open the connection to the serial port
        ser = serial.Serial(port, baud_rate, timeout=1)
        print(f"Connected to {port} a {baud_rate} baudios.")

        while True:
            # Read the message from standard input
            mensaje = input("Write the message to send (or 'exit' to exit): ")

            if mensaje.lower() == 'exit':
                print("Closing the connection.")
                break

            # Send the message through the serial port
            ser.write(mensaje.encode('utf-8'))
            print(f"Message sent: {mensaje}")

            # Read the response from the serial port
            time.sleep(0.1)  # Wait a moment to allow the response
            if ser.in_waiting > 0:
                response = ser.read(ser.in_waiting).decode('utf-8')
                print(f"Response received: {response}")

        # Close the connection to the serial port
        ser.close()

    except serial.SerialException as e:
        print(f"Could not open port {port}: {e}")

if __name__ == "__main__":
    main()
