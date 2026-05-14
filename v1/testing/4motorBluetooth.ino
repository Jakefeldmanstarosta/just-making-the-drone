#include <ESP32Servo.h>
#include "BluetoothSerial.h"

// PIN DEFINITIONS
#define M1_PIN 12
#define M2_PIN 26  // Moved from 13/25 to 26
#define M3_PIN 14
#define M4_PIN 27
#define BLUE_LED 2

BluetoothSerial SerialBT;
Servo m1, m2, m3, m4;

// 18V Safety Settings
int currentThrottle = 900; 
int targetThrottle = 900;  
int rampSpeed = 1; 

void setup() {
  Serial.begin(115200);
  pinMode(BLUE_LED, OUTPUT);
  SerialBT.begin("Esp32-Drone-BT");

  // Allocate 4 separate timers for maximum stability
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);
  
  // Attach all 4 motors with the 900us "Deep Zero" floor
  m1.attach(M1_PIN, 900, 2000);
  m2.attach(M2_PIN, 900, 2000);
  m3.attach(M3_PIN, 900, 2000);
  m4.attach(M4_PIN, 900, 2000);

  // Send the "Safe" signal immediately
  m1.writeMicroseconds(900);
  m2.writeMicroseconds(900);
  m3.writeMicroseconds(900);
  m4.writeMicroseconds(900);
  
  // Extra long 5-second wait for 18V ESC initialization
  delay(5000); 
  Serial.println("18V System Initialized on Pins 12, 26, 14, 27.");
}

void loop() {
  if (SerialBT.available()) {
    char cmd = SerialBT.read();
    
    if (cmd == '0') targetThrottle = 900;  // FULL STOP
    if (cmd == '1') targetThrottle = 1080; // ULTRA-LOW IDLE (Safe for 18V)
    if (cmd == '2') targetThrottle = 1150; // LOW SPIN
    if (cmd == '3') targetThrottle = 1250; // MEDIUM SPIN
    
    SerialBT.print("18V Target: ");
    SerialBT.println(targetThrottle);
  }

  // Linear Ramping
  if (currentThrottle < targetThrottle) {
    currentThrottle += rampSpeed;
  } else if (currentThrottle > targetThrottle) {
    currentThrottle -= rampSpeed;
  }

  // Simultaneous Command to all 4 ESCs
  m1.writeMicroseconds(currentThrottle);
  m2.writeMicroseconds(currentThrottle);
  m3.writeMicroseconds(currentThrottle);
  m4.writeMicroseconds(currentThrottle);

  // Status LED
  digitalWrite(BLUE_LED, (currentThrottle > 950) ? HIGH : LOW);
  delay(20); 
}
