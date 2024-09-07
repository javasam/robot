#include <Servo.h>
#define servoPin 10  //The pin of servo
#define leftMotorCtrl 4  //Define the direction control pin of the left motor
#define leftMotorPwr 6   //Define the PWM control pin of the left motor
#define rightMotorCtrl 2  //Define the direction control pin of the right motor
#define rightMotorPwr 5   //Define the PWM control pin of the right motor
#define Trig 12           //display
#define Echo 13           //ultrasonic sensor
float distance;
int a, a1, a2;
Servo myservo;

void setup() {
  Serial.begin(9600);
  myservo.attach(10);
  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);
  pinMode(leftMotorCtrl, OUTPUT);
  pinMode(leftMotorPwr, OUTPUT);
  pinMode(rightMotorCtrl, OUTPUT);
  pinMode(rightMotorPwr, OUTPUT);
  myservo.write(90);
  delay(500); //delay in 500ms
}

void loop() {
  a = checkdistance("initial");  //Assign the distance to the front detected by ultrasonic sensor to the variable a

  if (a < 10) {//When the distance to the front is less than 20cm
    Car_Stop();  //The robot stops
    delay(500); //delay in 500ms
    myservo.write(180);  //Ultrasonic pan-tilt turns left
    delay(1000); //delay in 500ms
    a1 = checkdistance("a1");  //Assign the distance to the left detected by ultrasonic sensor to the variable a1
    delay(100); //read value
    myservo.write(0);
    delay(1500); //delay in 500ms
    a2 = checkdistance("a2"); //Assign the distance to the right detected by ultrasonic sensor to the variable a2
    delay(100); //read value
    
    myservo.write(90);
    delay(500);
    if (a1 > a2) { //When the distance to the left is bigger than to the right
      Car_left();  //The robot turns left
      delay(700);  //turn left700ms
    } else if (a1 < a2 ) {
      Car_right(); //It turns left for 700ms
      delay(700);
    } else {
      //Car_back();
      delay(1700);
    }
  } 
  else { //When the distance to the front is >=20c，the robot moves forward
    Car_front(); //go front
  }
}

void Car_front() {
  digitalWrite(rightMotorCtrl, HIGH);
  analogWrite(rightMotorPwr, 650);
  digitalWrite(leftMotorCtrl, HIGH);
  analogWrite(leftMotorPwr, 660);
}

void Car_back() {
  digitalWrite(rightMotorCtrl, LOW);
  analogWrite(rightMotorPwr, 100);
  digitalWrite(leftMotorCtrl, LOW);
  analogWrite(leftMotorPwr, 100);
}

void Car_left() {
  //delay(100);
  //Car_back();
  //delay(500);
  digitalWrite(rightMotorCtrl, HIGH);
  analogWrite(rightMotorPwr, 100);
  digitalWrite(leftMotorCtrl, LOW);
  analogWrite(leftMotorPwr, 365);
}

void Car_right() {
  //delay(100);
  //Car_back();
  //delay(500);
  digitalWrite(rightMotorCtrl, LOW);
  analogWrite(rightMotorPwr, 365);
  digitalWrite(leftMotorCtrl, HIGH);
  analogWrite(leftMotorPwr,100);
}

void Car_Stop() {
  digitalWrite(rightMotorCtrl, LOW);
  analogWrite(rightMotorPwr, 0);
  digitalWrite(leftMotorCtrl, LOW);
  analogWrite(leftMotorPwr, 0);
}

//The function controls ultrasonic sound
float checkdistance(String from) {
  float distance;
  digitalWrite(Trig, LOW);
  delayMicroseconds(2);
  digitalWrite(Trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(Trig, LOW);
  distance = pulseIn(Echo, HIGH) / 58.20;  //The 58.20 here comes from 2*29.1=58.2
  Serial.print("distance " + from + ": ");
  Serial.print(distance);
  Serial.println();
  delay(10);
  return distance;
}
//****************************************************************
