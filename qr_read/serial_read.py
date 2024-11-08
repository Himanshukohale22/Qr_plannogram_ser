import serial

# Open the serial port (replace with your actual port and settings)
ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)  # Adjust port and baudrate

# Read data from the serial port
while True:
    if ser.in_waiting > 0:  # Check if there's data waiting
        data = ser.readline().decode('utf-8').strip()  # Read one line, decode from bytes to string
        print(f"Received: {data}")  # Print received data
