import serial

# Define the serial port and baud rate
ser = serial.Serial('/dev/cu.usbmodem86000301', 115200)  # Replace with your Teensy's port

try:
    while True:
        # Send a command to the Teensy
        command = input("Enter a command: ")
        ser.write(command.encode('utf-8'))

        # Receive and print the response from Teensy
        response = ser.readline().decode('utf-8').strip()
        print(f"Teensy Response: {response}")

except KeyboardInterrupt:
    print("Communication stopped.")
    ser.close()
