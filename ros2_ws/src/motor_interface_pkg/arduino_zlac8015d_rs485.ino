// Arduino sketch for ZLAC8015D RS485 control from ROS2
// Place this file in your motor_interface_pkg folder for reference
//
// == Required Libraries ==
// Install ModbusMaster from Arduino Library Manager
#include <ModbusMaster.h>

// == USER: Change these pins/settings as needed ==
#define RS485_DE_RE_PIN 2        // Pin to control RS485 DE/RE (direction)
#define ZLAC_MODBUS_ID 1         // Modbus address of your ZLAC8015D (check DIP switches/manual)
#define RS485_BAUDRATE 9600      // Baudrate for ZLAC8015D (default 9600, check your driver)
// If using Arduino Uno/Nano, use SoftwareSerial instead of Serial1 (see comments below)

ModbusMaster node;

void preTransmission() {
  digitalWrite(RS485_DE_RE_PIN, 1); // Set DE/RE high (transmit mode)
}

void postTransmission() {
  digitalWrite(RS485_DE_RE_PIN, 0); // Set DE/RE low (receive mode)
}

void setup() {
  Serial.begin(115200);    // USB serial to PC/ROS
  Serial1.begin(RS485_BAUDRATE); // RS485 to ZLAC8015D
  pinMode(RS485_DE_RE_PIN, OUTPUT);
  digitalWrite(RS485_DE_RE_PIN, 0);

  node.begin(ZLAC_MODBUS_ID, Serial1);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);
}

void loop() {
  // 1. Read command from ROS/PC
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    if (cmd.startsWith("R")) {
      int commaIdx = cmd.indexOf(',');
      if (commaIdx > 1) {
        int left = cmd.substring(1, commaIdx).toInt();
        int right = cmd.substring(commaIdx + 1).toInt();
        setZLACSpeed(1, left);  // Channel 1 (left motor)
        setZLACSpeed(2, right); // Channel 2 (right motor)
      }
    }
  }

  // 2. Periodically read actual speed from ZLAC8015D and send to ROS/PC
  static unsigned long lastSend = 0;
  if (millis() - lastSend > 50) {
    lastSend = millis();
    int left_rpm = readZLACSpeed(1);  // channel 1
    int right_rpm = readZLACSpeed(2); // channel 2
    Serial.print("E:");
    Serial.print(left_rpm);
    Serial.print(",");
    Serial.println(right_rpm);
  }
}

// Send speed command to ZLAC8015D
void setZLACSpeed(int channel, int rpm) {
  // == USER: Confirm register addresses in your ZLAC8015D manual ==
  uint16_t reg = (channel == 1) ? 0x2002 : 0x2102; // 0x2002 for M1, 0x2102 for M2 (typical)
  node.writeSingleRegister(reg, rpm);
}

// Read speed from ZLAC8015D
int readZLACSpeed(int channel) {
  // == USER: Confirm register addresses in your ZLAC8015D manual ==
  uint16_t reg = (channel == 1) ? 0x200C : 0x210C; // 0x200C for M1, 0x210C for M2 (typical)
  node.readHoldingRegisters(reg, 1);
  return (int16_t)node.getResponseBuffer(0); // Cast to signed int
}

// == If using Arduino Uno/Nano, replace Serial1 with SoftwareSerial ==
// #include <SoftwareSerial.h>
// SoftwareSerial rs485Serial(10, 11); // RX, TX (change pins as needed)
// In setup(): rs485Serial.begin(RS485_BAUDRATE); node.begin(ZLAC_MODBUS_ID, rs485Serial);
// Replace all Serial1 with rs485Serial 