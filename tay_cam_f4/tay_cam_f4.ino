#include <Ps3Controller.h>
#include "Ps3.h"
//HardwareSerial Serial2(2);  // Sử dụng UART2

uint16_t button;
unsigned long timeJump = 0;
bool connect = false;
bool ledState = false;
// xử lý ngắt uart
void rx_tx() {
  if (Serial2.available() > 0) {
    //Serial.print("da nhan");
    char receive = Serial2.read();
    if (receive == 'o') {
      Serial.println(receive);
      uint8_t Send_Data[2];
      // gửi giá trị nút bấm
      Send_Data[0] = (button >> 8) & 0xFF;  // lấy bit cao  (trái)
      Send_Data[1] = button & 0xFF;         // lấy bit thấp (phải)

      Serial.print("button : ");
      Serial.println((uint16_t)((Send_Data[0] << 8) | Send_Data[1]));  // gửi 2 byte
      for (uint8_t i = 0; i < 2; i++) {
        Serial2.write(Send_Data[i]);  // send to stm32
      }
    }
  }
}
void notify() {
  // up
  if (Ps3.data.button.up) {
    button |= 0x01;
    digitalWrite(2, 1);
  } else {
    button &= ~0x01;
    digitalWrite(2, 0);
  }
  // down
  if (Ps3.data.button.down) {
    button |= 0x02;
    digitalWrite(2, 1);
  } else {
    button &= ~0x02;
    digitalWrite(2, 0);
  }
  // right
  if (Ps3.data.button.right) {
    button |= 0x04;
    digitalWrite(2, 1);
  } else {
    button &= ~0x04;
    digitalWrite(2, 0);
  }
  // left
  if (Ps3.data.button.left) {
    button |= 0x08;
    digitalWrite(2, 1);
  } else {
    button &= ~0x08;
    digitalWrite(2, 0);
  }
  // L1
  if (Ps3.data.button.l1) {
    button |= 0x10;
    digitalWrite(2, 1);
  } else {
    button &= ~0x10;
    digitalWrite(2, 0);
  }
  // R1
  if (Ps3.data.button.r1) {
    button |= 0x20;
    digitalWrite(2, 1);
  } else {
    button &= ~0x20;
    digitalWrite(2, 0);
  }
  // L2
  if (Ps3.data.button.l2) {
    button |= 0x40;
    digitalWrite(2, 1);
  } else {
    button &= ~0x40;
    digitalWrite(2, 0);
  }
  // R2
  if (Ps3.data.button.r2) {
    button |= 0x80;
    digitalWrite(2, 1);
  } else {
    button &= ~0x80;
    digitalWrite(2, 0);
  }
  // cross
  if (Ps3.data.button.cross) {
    button |= 0x100;
    digitalWrite(2, 1);
  } else {
    button &= ~0x100;
    digitalWrite(2, 0);
  }
  // cricle
  if (Ps3.data.button.circle) {
    button |= 0x200;
    digitalWrite(2, 1);
  } else {
    button &= ~0x200;
    digitalWrite(2, 0);
  }
  // square
  if (Ps3.data.button.square) {
    button |= 0x400;
    digitalWrite(2, 1);
  } else {
    button &= ~0x400;
    digitalWrite(2, 0);
  }
  // triangle
  if (Ps3.data.button.triangle) {
    button |= 0x800;
    digitalWrite(2, 1);
  } else {
    button &= ~0x800;
    digitalWrite(2, 0);
  }
  // select
  if (Ps3.data.button.select) {
    button |= 0x2000;
    digitalWrite(2, 1);
  } else {
    button &= ~0x2000;
    digitalWrite(2, 0);
  }
  // start
  if (Ps3.data.button.start) {

    button |= 0x4000;
    digitalWrite(2, 1);
  } else {
    digitalWrite(2, 0);
    button &= ~0x4000;
  }
  // L3
  if (Ps3.data.button.l3) {
    button |= 0x1000;
    digitalWrite(2, 1);
  } else {
    button &= ~0x1000;
    digitalWrite(2, 0);
  }
  // R3
  if (Ps3.data.button.r3) {
    button |= 0x8000;
    digitalWrite(2, 1);
  } else {
    button &= ~0x8000;
    digitalWrite(2, 0);
  }
}
void onConnect() {
  Serial.println("Connected!.");
  connect = true;
}
void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, 16, 17);  // RX ------- TX   // later 13,19
  //Serial2.onReceive(onReceive);
  Ps3.attach(notify);
  Ps3.attachOnConnect(onConnect);
  Ps3.begin("00:00:00:15:12:05");  //d4:d4:da:e3:a3:c4 14:2b:2f:c5:76:e8
  //Ps3.begin("00:05:01:04:00:00");
  //Ps3.begin("f8:b3:b7:45:40:16");
 // pinMode(2,OUTPUT);
}
void loop() {
  //Serial2.println("hello_main");
  if (connect) {
    /*if (millis() - timeJump > 1000) {
      ledState = !ledState;
      digitalWrite(2, ledState);
      timeJump = millis();
    }
    */
  }
  rx_tx();
}