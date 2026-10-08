#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>

// Virtual ESP32 Rover wiring model
// ESP32 -> Motor Driver -> Left/Right DC motors
// ESP32 -> HC-SR04-style ultrasonic: TRIG/ECHO
// In the desktop simulator these pins are virtual, but the same command/sensor
// contract is used by claude-robot.
//
// GPIO map:
// TRIG 5, ECHO 18
// MOTOR_L_PWM 25, MOTOR_L_DIR 26
// MOTOR_R_PWM 27, MOTOR_R_DIR 14

const int TRIG_PIN=5,ECHO_PIN=18;
const int MOTOR_L_PWM=25,MOTOR_L_DIR=26,MOTOR_R_PWM=27,MOTOR_R_DIR=14;

struct MotorDriverState { int left=0; int right=0; };
MotorDriverState motors;

void applyMotorDriver(int left,int right){
  motors.left=constrain(left,-100,100);
  motors.right=constrain(right,-100,100);
  // Real ESP32 implementation would map these values to PWM + DIR pins.
  analogWrite(MOTOR_L_PWM,abs(motors.left)*255/100);
  digitalWrite(MOTOR_L_DIR,motors.left>=0?HIGH:LOW);
  analogWrite(MOTOR_R_PWM,abs(motors.right)*255/100);
  digitalWrite(MOTOR_R_DIR,motors.right>=0?HIGH:LOW);
}

float readUltrasonicCm(){
  digitalWrite(TRIG_PIN,LOW); delayMicroseconds(2);
  digitalWrite(TRIG_PIN,HIGH); delayMicroseconds(10);
  digitalWrite(TRIG_PIN,LOW);
  unsigned long us=pulseIn(ECHO_PIN,HIGH,30000);
  if(!us)return 400.0f;
  return us*0.0343f/2.0f;
}

void setup(){
  Serial.begin(115200);
  pinMode(TRIG_PIN,OUTPUT); pinMode(ECHO_PIN,INPUT);
  pinMode(MOTOR_L_PWM,OUTPUT); pinMode(MOTOR_L_DIR,OUTPUT);
  pinMode(MOTOR_R_PWM,OUTPUT); pinMode(MOTOR_R_DIR,OUTPUT);
  applyMotorDriver(0,0);
}

void loop(){
  // Command protocol is intentionally the same contract used by claude-robot.
  // Desktop virtual physics replaces the actual GPIO effect and supplies the
  // same ultrasonic measurement from the virtual map.
  float frontDistanceCm=readUltrasonicCm();
  (void)frontDistanceCm;
  delay(100);
}