#include <Servo.h>          // Servo motor library
#include <NewPing.h>        // Ultrasonic sensor function library

// L298N control pins
const int LeftMotorForward = 3;
const int LeftMotorBackward = 6;
const int RightMotorForward = 5;
const int RightMotorBackward = 11;
const int buzzerPin = 13;   // Buzzer pin
const int analogSensorPin = A0;  // Analog sensor pin

// Ultrasonic sensor pins
#define trig_pin A1 // analog input 1
#define echo_pin A2 // analog input 2

#define maximum_distance 200
boolean goesForward = false;
int distance = 100;

// Define motor speed (PWM value: 0-255)
const int motorSpeed = 120;

NewPing sonar(trig_pin, echo_pin, maximum_distance); // sensor function
Servo servo_motor; // servo name

void setup() {
  // Initialize motor control pins
  pinMode(RightMotorForward, OUTPUT);
  pinMode(LeftMotorForward, OUTPUT);
  pinMode(LeftMotorBackward, OUTPUT);
  pinMode(RightMotorBackward, OUTPUT);
  
  // Initialize buzzer and analog sensor pins
  pinMode(buzzerPin, OUTPUT);  
  pinMode(analogSensorPin, INPUT);  

  // Attach the servo motor
  servo_motor.attach(10);

  // Center the servo
  servo_motor.write(115);
  delay(2000);

  // Initialize distance readings
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
}

void loop() {
  // Read the analog sensor value
  int sensorValue = analogRead(analogSensorPin);
  
  // If the sensor value is below 200, stop the bot and sound the buzzer for 5 seconds
  if (sensorValue < 200) {
    moveStop();
    digitalWrite(buzzerPin, HIGH);  // Turn on buzzer
    delay(5000);  // Buzzer on for 5 seconds
    digitalWrite(buzzerPin, LOW);   // Turn off buzzer
  } else {
    digitalWrite(buzzerPin, LOW);   // Ensure buzzer is off
    
    // Proceed with obstacle avoidance logic
    delay(50);

    if (distance <= 45) {
      moveStop();
      delay(300);
      moveBackward();
      delay(400);
      moveStop();
      delay(300);
      int distanceRight = lookRight();
      delay(300);
      int distanceLeft = lookLeft();
      delay(300);

      if (distanceRight >= distanceLeft) {
        turnRight();
      } else {
        turnLeft();
      }
    } else {
      moveForward();
    }
    
    // Continuously update distance
    distance = readPing();
  }
}

int lookRight() {
  servo_motor.write(50);
  delay(500);
  int distance = readPing();
  delay(100);
  servo_motor.write(115);  // Center the servo back
  return distance;
}

int lookLeft() {
  servo_motor.write(170);
  delay(500);
  int distance = readPing();
  delay(100);
  servo_motor.write(115);  // Center the servo back
  return distance;
}

int readPing() {
  delay(70);
  int cm = sonar.ping_cm();
  if (cm == 0) {
    cm = 250;  // If no reading, return max distance
  }
  return cm;
}

void moveStop() {
  analogWrite(RightMotorForward, 0);
  analogWrite(LeftMotorForward, 0);
  analogWrite(RightMotorBackward, 0);
  analogWrite(LeftMotorBackward, 0);
}

void moveForward() {
  if (!goesForward) {
    goesForward = true;
    analogWrite(LeftMotorForward, motorSpeed);
    analogWrite(RightMotorForward, motorSpeed);
    analogWrite(LeftMotorBackward, 0);
    analogWrite(RightMotorBackward, 0);
  }
}

void moveBackward() {
  goesForward = false;
  analogWrite(LeftMotorBackward, motorSpeed);
  analogWrite(RightMotorBackward, motorSpeed);
  analogWrite(LeftMotorForward, 0);
  analogWrite(RightMotorForward, 0);
}

void turnRight() {
  analogWrite(LeftMotorForward, motorSpeed);
  analogWrite(RightMotorBackward, motorSpeed);
  analogWrite(LeftMotorBackward, 0);
  analogWrite(RightMotorForward, 0);
  delay(2000);
  moveForward();  // Move forward after turning
}

void turnLeft() {
  analogWrite(LeftMotorBackward, motorSpeed);
  analogWrite(RightMotorForward, motorSpeed);
  analogWrite(LeftMotorForward, 0);
  analogWrite(RightMotorBackward, 0);
  delay(2000);
  moveForward();  // Move forward after turning
}
