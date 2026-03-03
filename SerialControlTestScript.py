import serial
import time

ser = serial.Serial('/dev/cu.usbserial-10', 115200)  # Update port
time.sleep(2)  # Let Arduino reset

while True:
    try:
        value = int(input("Enter a number (0-255) to send over serial (or -1 to quit): "))
        if value == -1:
            break
        if 0 <= value <= 255:
            ser.write(bytes([value]))
        else:
            print("Please enter a value between 0 and 255.")
    except ValueError:
        print("Invalid input. Please enter an integer.")

ser.close()