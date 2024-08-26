#include <Servo.h>         
#include <NewPing.h>        

const int LeftMotorForward = 7;
const int LeftMotorBackward = 6;
const int RightMotorForward = 5;
const int RightMotorBackward = 4;

const int ForwardLED = 8;
const int BackwardLED = 9;
const int LeftLED = 11;
const int RightLED = 12;
const int LeftSensorLED = 13;
const int RightSensorLED = 3;

const int LeftIRSensor = A3;  
const int RightIRSensor = A4; 
const int Buzzer = 2;         

#define trig_pin A1 
#define echo_pin A2 

#define maximum_distance 200
boolean goesForward = false;
int distance = 100;

NewPing sonar(trig_pin, echo_pin, maximum_distance); 
Servo servo_motor; 

void setup() {

  pinMode(RightMotorForward, OUTPUT);
  pinMode(LeftMotorForward, OUTPUT);
  pinMode(LeftMotorBackward, OUTPUT);
  pinMode(RightMotorBackward, OUTPUT);
  pinMode(ForwardLED, OUTPUT);
  pinMode(BackwardLED, OUTPUT);
  pinMode(LeftLED, OUTPUT);
  pinMode(RightLED, OUTPUT);
  pinMode(LeftSensorLED, OUTPUT);
  pinMode(RightSensorLED, OUTPUT);
  
  pinMode(LeftIRSensor, INPUT);  
  pinMode(RightIRSensor, INPUT); 
  pinMode(Buzzer, OUTPUT);       

  servo_motor.attach(10); 

  servo_motor.write(115);
  delay(2000);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
}

void loop() {

  int distanceRight = 0;
  int distanceLeft = 0;
  delay(50);

  int fireDetected = checkFire(); 
  
  if (fireDetected) {
    moveStop(); 
    delay(2000); 
    moveBackward(); 
    buzzBuzzer(); 
    delay(1000); 
    moveStop(); 
    delay(1000); 
  } 
  else if (distance <= 45) {
    moveStop();
    delay(300);
    moveBackward();
    delay(400);
    moveStop();
    delay(300);
    distanceRight = lookRight();
    delay(300);
    distanceLeft = lookLeft();
    delay(300);

    if (distance >= distanceLeft) {
      turnRight();
      moveStop();
    } else {
      turnLeft();
      moveStop();
    }
  } else {
    moveForward();
  }
  distance = readPing();
}

int lookRight() {  
  digitalWrite(RightSensorLED, HIGH);
  delay(200);
  digitalWrite(RightSensorLED, LOW);
  servo_motor.write(50);
  delay(500);
  int distance = readPing();
  delay(100);
  servo_motor.write(115);
  return distance;
}

int lookLeft() {
  digitalWrite(LeftSensorLED, HIGH);
  delay(500);
  digitalWrite(LeftSensorLED, LOW);
  servo_motor.write(170);
  delay(500);
  int distance = readPing();
  delay(100);
  servo_motor.write(115);
  return distance;
  delay(100);
}

int readPing() {
  delay(70);
  int cm = sonar.ping_cm();
  if (cm == 0) {
    cm = 250;
  }
  return cm;
}

void moveStop() {
  digitalWrite(RightMotorForward, LOW);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(ForwardLED, LOW);
  digitalWrite(BackwardLED, LOW);
  digitalWrite(LeftLED, LOW);
  digitalWrite(RightLED, LOW);
  digitalWrite(LeftSensorLED, LOW);
  digitalWrite(RightSensorLED, LOW);
}

void moveForward() {
  if (!goesForward) {
    goesForward = true;
    digitalWrite(LeftMotorForward, HIGH);
    digitalWrite(RightMotorForward, HIGH);
    digitalWrite(LeftMotorBackward, LOW);
    digitalWrite(RightMotorBackward, LOW);
    digitalWrite(ForwardLED, HIGH);
  }
}

void moveBackward() {
  goesForward = false;
  digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorBackward, HIGH);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorForward, LOW);
  digitalWrite(BackwardLED, HIGH);
}

void turnRight() {
  digitalWrite(RightLED, HIGH);
  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorBackward, HIGH);
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorForward, LOW);
  delay(2000);
  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorForward, HIGH);
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorBackward, LOW);
}

void turnLeft() {
  digitalWrite(LeftLED, HIGH);
  digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorForward, HIGH);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);
  delay(2000);
  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorForward, HIGH);
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorBackward, LOW);
}
#Hinmanshu LOW,LOW
int checkFire() {
  int leftIRValue = digitalRead(LeftIRSensor);
  int rightIRValue = digitalRead(RightIRSensor);
  if (leftIRValue == HIGH || rightIRValue == HIGH) {
    return 1;
  }
  return 0;
}

void buzzBuzzer() {
  digitalWrite(Buzzer, HIGH);
  delay(10000);
  digitalWrite(Buzzer, LOW);
}
