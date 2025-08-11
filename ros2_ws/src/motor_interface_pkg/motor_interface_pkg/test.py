from pymodbus.client.serial import ModbusSerialClient as ModbusClient

# Connect to the motor driver
client = ModbusClient(
    # method='rtu',
    port='/dev/motor_usb',    # adjust as needed
    baudrate=115200,
    stopbits=1,
    bytesize=8,
    parity='N',
    timeout=1
)

unit_id = 1  # Modbus slave ID of ZLAC8015D
client.connect()

# --- Enable the motor ---
client.write_register(address=0x200E, value=8, device_id=unit_id)  # enable both motors

# --- Set motor speeds (in RPM) ---
motor1_speed = 0   # positive = forward, negative = backward
motor2_speed = 0

while motor1_speed != 50:
    client.write_register(address=0x2088, value=motor1_speed & 0xFFFF, device_id=unit_id)  # motor 1
    client.write_register(address=0x2089, value=motor2_speed & 0xFFFF, device_id=unit_id)  # motor 2
    motor1_speed += 1
    motor2_speed -= 1

    # --- Read actual speeds ---
    resp1 = client.read_holding_registers(address=0x20AB, count=1, device_id=unit_id)
    resp2 = client.read_holding_registers(address=0x20AC, count=1, device_id=unit_id)
    print("input: ", motor1_speed)
    if not resp1.isError():
        speed1 = resp1.registers[0]
        if speed1 > 32767:
            speed1 -= 65536
        print("Motor 1 speed:", speed1)

    if not resp2.isError():
        speed2 = resp2.registers[0]
        if speed2 > 32767:
            speed2 -= 65536
        print("Motor 2 speed:", speed2)

while motor1_speed != 0:
    client.write_register(address=0x2088, value=motor1_speed & 0xFFFF, device_id=unit_id)  # motor 1
    client.write_register(address=0x2089, value=motor2_speed & 0xFFFF, device_id=unit_id)  # motor 2
    motor1_speed -= 1
    motor2_speed += 1

    # --- Read actual speeds ---
    resp1 = client.read_holding_registers(address=0x20AB, count=1, device_id=unit_id)
    resp2 = client.read_holding_registers(address=0x20AC, count=1, device_id=unit_id)
    print("input: ", motor1_speed)
    if not resp1.isError():
        speed1 = resp1.registers[0]
        if speed1 > 32767:
            speed1 -= 65536
        print("Motor 1 speed:", speed1)

    if not resp2.isError():
        speed2 = resp2.registers[0]
        if speed2 > 32767:
            speed2 -= 65536
        print("Motor 2 speed:", speed2)

# --- Stop the motors ---
client.write_register(address=0x2088, value=0, device_id=unit_id)
client.write_register(address=0x2089, value=0, device_id=unit_id)

# --- Disable motors ---
client.write_register(address=0x200E, value=0, device_id=unit_id)

client.close()
