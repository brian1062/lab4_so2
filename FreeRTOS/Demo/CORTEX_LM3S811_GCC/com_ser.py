import serial
import threading
import argparse
import time

def read_uart(ser, is_running):
    while is_running[0]:
        if ser.in_waiting > 0:
            time.sleep(0.15)
            response = ser.read(ser.in_waiting).decode('utf-8')
            print(f"{response}")

def main():
    # Set up the command-line argument
    parser = argparse.ArgumentParser(description="Send messages via UART to a specific device.")
    parser.add_argument("pts", type=int, help="Number of the pts port (e.g., 2 for /dev/pts/2)")
    args = parser.parse_args()

    # Configure the serial port
    port = f'/dev/pts/{args.pts}'
    baud_rate = 9600  # Adjust the baud rate as needed

    try:
        # Open the connection to the serial port
        ser = serial.Serial(port, baud_rate, bytesize=serial.EIGHTBITS, stopbits=serial.STOPBITS_ONE, parity=serial.PARITY_NONE, timeout=1)
        print(f"Connected to {port} at {baud_rate} baud.")

        is_running = [True] # Variable to control the loop

        # Create and start the reading thread
        reading_thread = threading.Thread(target=read_uart, args=(ser,is_running))
        reading_thread.daemon = True
        reading_thread.start()

        while True:
            # Read the message from standard input
            message = input("")#Enter the message to send (or 'exit' to quit): ")

            if message.lower() == 'exit':
                print("Closing the connection.")
                is_running[0] = False
                break

            # Send the message via the serial port
            ser.write(message.encode('utf-8'))
            # print(f"Message sent: {message}")

        reading_thread.join()
        # Close the connection to the serial port
        ser.close()

    except serial.SerialException as e:
        print(f"Could not open port {port}: {e}")

if __name__ == "__main__":
    main()