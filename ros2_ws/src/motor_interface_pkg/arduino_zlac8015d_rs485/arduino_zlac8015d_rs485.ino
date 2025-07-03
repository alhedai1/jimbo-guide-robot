#include <ModbusMaster.h>

#define MAX485_DE_RE 22  // RS485 제어 핀

ModbusMaster node;

void preTransmission() {
  digitalWrite(MAX485_DE_RE, HIGH);
}

void postTransmission() {
  digitalWrite(MAX485_DE_RE, LOW);
}

void setup() {
  pinMode(MAX485_DE_RE, OUTPUT);
  digitalWrite(MAX485_DE_RE, LOW);

  Serial.begin(115200);        // ROS 2와 통신
  while (!Serial);
  Serial.println("Arduino ready");
  Serial1.begin(115200);       // RS485 (ZLAC8015D)

  node.begin(1, Serial1);
  node.preTransmission(preTransmission);
  node.postTransmission(postTransmission);

  delay(500);

  // 속도 모드 설정
  if (node.writeSingleRegister(0x200D, 0x0003) == node.ku8MBSuccess)
    Serial.println("Speed mode set.");
  else
    Serial.println("❌ Failed to set speed mode.");

  if (node.writeSingleRegister(0x200E, 0x0008) == node.ku8MBSuccess)
    Serial.println("Motor enabled.");
  else
    Serial.println("❌ Failed to enable motor.");
  delay(200);
}

void loop() {
  // 1. ROS 2에서 RPM 명령 수신
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');  // 예: R120,-110
    input.trim();

    if (input.startsWith("R")) {
      // Serial.print("Got: "); Serial.println(input);
      input.remove(0, 1);  // 'R' 제거
      int commaIdx = input.indexOf(',');
      if (commaIdx > 0) {
        int16_t left_rpm = input.substring(0, commaIdx).toInt();
        int16_t right_rpm = input.substring(commaIdx + 1).toInt();

        node.writeSingleRegister(0x2088, left_rpm);   // Left motor
        node.writeSingleRegister(0x2089, right_rpm);  // Right motor
      }
    }
  }

  // 2. ZLAC8015D에서 실제 속도 읽기
  uint8_t result = node.readHoldingRegisters(0x20AB, 2);  // 0x20AB: left, 0x20AC: right
  if (result == node.ku8MBSuccess) {
    int16_t actual_left = (int16_t)node.getResponseBuffer(0);
    int16_t actual_right = (int16_t)node.getResponseBuffer(1);

    // 3. ROS 2로 응답 전송
    Serial.print("E:");
    Serial.print(actual_left);
    Serial.print(",");
    Serial.println(actual_right);
  }

  delay(50);  // 20Hz 주기
}



// #include <ModbusMaster.h>

// #define MAX485_DE_RE 22  // RS485 제어 핀 (DE/RE 공통)

// // Modbus 인스턴스 생성
// ModbusMaster node;

// void preTransmission() {
//   digitalWrite(MAX485_DE_RE, HIGH);
// }

// void postTransmission() {
//   digitalWrite(MAX485_DE_RE, LOW);
// }

// void setup() {
//   pinMode(MAX485_DE_RE, OUTPUT);
//   digitalWrite(MAX485_DE_RE, LOW);  // 초기엔 수신 모드

//   Serial.begin(115200);   // 디버깅용 시리얼
//   Serial1.begin(115200);  // ZLAC8015D 기본 보레이트

//   node.begin(1, Serial1);  // 슬레이브 주소 1
//   node.preTransmission(preTransmission);
//   node.postTransmission(postTransmission);

//   delay(500);

//   // 운전 모드 설정 (속도 모드: 0x0003)
//   if (node.writeSingleRegister(0x200D, 0x0003) == node.ku8MBSuccess) {
//     Serial.println("✅ Mode set to Speed Mode.");
//   } else {
//     Serial.println("❌ Failed to set speed mode.");
//   }
//   delay(100);

//   // 모터 Enable (0x200E ← 0x0008)
//   if (node.writeSingleRegister(0x200E, 0x0008) == node.ku8MBSuccess) {
//     Serial.println("✅ Motor enabled.");
//   } else {
//     Serial.println("❌ Motor enable failed.");
//   }

//   delay(500);
// }

// int speed = 0;
// bool increasing = true;

// void loop() {
//   // 1. 현재 속도 설정
//   if (node.writeSingleRegister(0x2088, -speed) == node.ku8MBSuccess && node.writeSingleRegister(0x2089, speed) == node.ku8MBSuccess) {
//     Serial.print("✅ Set speed to: ");
//     Serial.println(speed);
//   } else {
//     Serial.println("❌ Failed to set speed.");
//   }

//   delay(300);  // 모터 반응 대기

//   // 2. 실제 속도 읽기
//   uint8_t result = node.readHoldingRegisters(0x20AB, 2);
//   if (result == node.ku8MBSuccess) {
//     int16_t left = (int16_t)node.getResponseBuffer(0);
//     int16_t right = (int16_t)node.getResponseBuffer(1);

//     Serial.print("📈 Actual Left Speed: ");
//     Serial.print(left / 10.0);
//     Serial.print(" RPM | Right Speed: ");
//     Serial.print(right / 10.0);
//     Serial.println(" RPM");
//   } else {
//     Serial.print("❌ Speed read failed. Code: ");
//     Serial.println(result, HEX);
//   }

//   delay(300);

//   // 3. 속도 증가 또는 감소
//   if (increasing) {
//     speed++;
//     if (speed >= 200) {
//       increasing = false;  // 최고 속도 도달 → 감소로 전환
//     }
//   } else {
//     speed--;
//     if (speed <= 0) {
//       // 4. 정지 명령 전송
//       if (node.writeSingleRegister(0x200E, 0x0007) == node.ku8MBSuccess) {
//         Serial.println("🛑 Motor stop command sent.");
//       } else {
//         Serial.println("❌ Stop command failed.");
//       }
//       while (true)
//         ;  // 동작 종료
//     }
//   }
// }
