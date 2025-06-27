import serial
import time

ser = serial.Serial(port='COM3', baudrate=115200, timeout=1)
print(f"Serial port {ser.portstr} opened successfully.")

def read_serial_data():
    while True:
        if ser.in_waiting > 0:
            data = ser.readline().decode('utf-8').rstrip()
            print(f"Received: {data}")

def main():
    try:
        print("Starting to read from serial port...")
        read_serial_data()
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        ser.close()
        print("Serial port closed.")

# if __name__ == "__main__":
#     main()

ser.write(b'\r\n')
time.sleep(0.1)

ser.write(b'si\r\n')
time.sleep(0.1)

while True:
    line = ser.readline()
    if line:
        print(">>", line.decode(errors='ignore').strip())