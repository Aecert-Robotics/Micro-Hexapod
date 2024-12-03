#include <Servo.h>
#include "vectors.h"
#include "Arduino.h"
#include "NRF.h"

#define COXA_PIN 29 
#define FEMUR_PIN 30
#define TIBIA_PIN 31 

Servo coxaServo;
Servo femurServo;
Servo tibiaServo;
float a1 = 28.1; //coxa length
float a2 = 67.8; //femur length
float a3 = 117.5; //tibia length
float legLength = a1 + a2 + a3;

//float offsets[3] = {90, 50, -10};
float offsets[3] = {0, 50, 0};

void moveToPos(Vector3 pos);
void moveToPosSmooth(Vector3 startPos, Vector3 endPos, int steps, int delayTime);
int angleToMicroseconds(double angle);
void ManualInput();
void TraceASquare();
float floatMap(float x, float in_min, float in_max, float out_min, float out_max);
void MoveToPotHeight();

void setup() {
  Serial.begin(9600);
  coxaServo.attach(COXA_PIN);  
  femurServo.attach(FEMUR_PIN);  
  tibiaServo.attach(TIBIA_PIN);  

  Vector3 pos1 = Vector3(0, 0, 0);
  Vector3 pos2 = Vector3(0, a1 + a3, a2);
  moveToPosSmooth(pos1, pos2, 50, 20);

  setupNRF();

  delay(1000);
}

void loop() {
  //ManualInput();
  //TraceASquare();
  MoveToPotHeight();
  recieveNRFData();
}


void MoveToPotHeight() {
  int potVal = analogRead(A10);

  if(radio.available()){
    potVal = floatMap(dataPackage.joy1_Y, 0, 256, 0, 1023);
  }
  
  float z = floatMap(potVal, 0, 1023, -140, -10);
  Vector3 pos = Vector3(0, 120, z);
  moveToPos(pos);
}

void TraceASquare() {
  Vector3 pos1 = Vector3(-60, 110, -80);
  Vector3 pos2 = Vector3(-60, 110, 30);
  Vector3 pos3 = Vector3(60, 110, 30);
  Vector3 pos4 = Vector3(60, 110, -80);

  int steps = 25;
  int pointDelay = 250;
  int delayTime = 5;

  moveToPosSmooth(pos1, pos2, steps, delayTime);
  delay(pointDelay);
  moveToPosSmooth(pos2, pos3, steps, delayTime);
  delay(pointDelay);
  moveToPosSmooth(pos3, pos4, steps, delayTime);
  delay(pointDelay);
  moveToPosSmooth(pos4, pos1, steps, delayTime);
  delay(pointDelay);
}

void ManualInput(){
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    float x, y, z;
    sscanf(input.c_str(), "%f %f %f", &x, &y, &z);
    
    Vector3 pos = Vector3(x, y, z);
    Serial.println("Moving to position: " + pos.toString());
    moveToPos(pos);
  }
  delay(100); // Small delay to avoid overwhelming the serial buffer
}

void moveToPos(Vector3 pos) {

  float dis = Vector3(0, 0, 0).distanceTo(pos);
  if (dis > legLength) {
    Serial.println("Position out of reach");
    return;
  }

  float x = pos.x;
  float y = pos.y;
  float z = pos.z;

  float o1 = offsets[0];
  float o2 = offsets[1];
  float o3 = offsets[2];

  float theta1 = atan2(y, x) * (180 / PI) + o1;  // base angle
  float l = sqrt(x * x + y * y);                 // x and y extension
  float l1 = l - a1;
  float h = sqrt(l1 * l1 + z * z);

  float phi1 = acos(constrain((pow(h, 2) + pow(a2, 2) - pow(a3, 2)) / (2 * h * a2), -1, 1));
  float phi2 = atan2(z, l1);
  float theta2 = (phi1 + phi2) * 180 / PI + o2;
  float phi3 = acos(constrain((pow(a2, 2) + pow(a3, 2) - pow(h, 2)) / (2 * a2 * a3), -1, 1));
  float theta3 = (phi3 * 180 / PI) + o3;

  coxaServo.write(180 - theta1);
  femurServo.write(180 - theta2);
  tibiaServo.write(theta3);
  //Serial.println("Theta1: " + String(theta1) + " Theta2: " + String(theta2) + " Theta3: " + String(theta3));
  return;
}

void moveToPosSmooth(Vector3 startPos, Vector3 endPos, int steps, int delayTime) {
  for (int i = 0; i <= steps; i++) {
    float t = (float)i / (float)steps;
    Vector3 intermediatePos = Vector3(
      startPos.x + t * (endPos.x - startPos.x),
      startPos.y + t * (endPos.y - startPos.y),
      startPos.z + t * (endPos.z - startPos.z)
    );
    moveToPos(intermediatePos);
    delay(delayTime);
  }
}

float floatMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}