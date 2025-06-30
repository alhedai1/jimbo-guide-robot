import serial
import time

# tag = COM3
# anchor0 = COM4

def request_sensor_data():
    tag.write(b'reset\r')
    anchor0.write(b'reset\r')

    time.sleep(1)

    tag.write(b'\r')
    anchor0.write(b'\r')
    tag.write(b'\r')
    anchor0.write(b'\r')

    time.sleep(1)

    tag.write(b'lec\r')

def read_sensor_data(serial_port):
    string = tag.readline().decode('utf-8').strip()



def main():
    global tag, anchor0
    tag = serial.Serial('COM3', baudrate=115200, timeout=1)
    anchor0 = serial.Serial('COM4', baudrate=115200, timeout=1)
    time.sleep(2)  # Allow time for the serial connection to establish
    
    request_sensor_data()

# tag.write(b'si\r')
# time.sleep(0.5)

if __name__ == "__main__":
    main()