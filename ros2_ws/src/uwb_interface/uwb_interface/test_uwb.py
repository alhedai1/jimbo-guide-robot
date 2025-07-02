import serial
import time

def request_sensor_data():
    tag.write(b'reset\r')

    time.sleep(1)

    tag.write(b'\r')
    tag.write(b'\r')

    time.sleep(1)

    tag.write(b'lec\r')

def read_sensor_data(serial_port):
    print("Reading sensor data...")
    while True:
        try:
            line = serial_port.readline()
            if not line:
                break
            decoded_line = line.decode('utf-8', errors='replace').strip()
            if decoded_line:
                print(f"Received: {decoded_line}")
        except KeyboardInterrupt:
            print("Interrupted by user.")
            break
        except Exception as e:
            print(f"Error reading from serial: {e}")
            break



def main():
    serial_port = serial.Serial('/dev/uwb_front_left', baudrate=115200, timeout=1)
    time.sleep(2)  # Allow time for the serial connection to establish

    global tag
    tag = serial_port  # For compatibility with request_sensor_data

    request_sensor_data()

    read_sensor_data(serial_port)

if __name__ == "__main__":
    main()