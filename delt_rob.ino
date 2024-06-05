#include "deltain.h"
#include "step.h"
#include <SPI.h>
#include <LoRa.h>

// ?#include <Servo.h>


#define servoPin = 2;
#define ss 5
#define rst 14
#define dio0 12
// Se?rvo servo1;

#define pi = 3.141592653;    // PI

void setup() {
  motoroff();
  Serial.begin(115200);

  for (int i = 0; i < 3; i++) {
    pinMode(endStop[i],INPUT_PULLUP);
  }

  // servo1.attach(servoPin);
  // BTS_ON();
  // TCP_ON();
  while (!Serial);
  Serial.println("LoRa Receiver");

  //setup LoRa transceiver module
  LoRa.setPins(ss, rst, dio0);
  
  //replace the LoRa.begin(---E-) argument with your location's frequency 
  //433E6 for Asia
  //866E6 for Europe
  //915E6 for North America
  while (!LoRa.begin(433E6)) {
    Serial.println(".");
    delay(500);
  }
   // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  LoRa.setSyncWord(0xF3);
  Serial.println("LoRa Initializing OK!");
}

void loop() {
  if (Serial.available()) {
    String received = Serial.readStringUntil('\n');
    Serial.println(received);
    command(received);
}  
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    while (LoRa.available()) {
      String loD = LoRa.readString();
      Serial.println(loD);
      command(loD);
    }
  }
}
