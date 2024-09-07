#include <Servo.h>
#define servoPin 10  //The pin of servo
int a, a1, a2;
#define ML_Ctrl 4  //Define the direction control pin of the left motor
#define ML_PWM 6   //Define the PWM control pin of the left motor
#define MR_Ctrl 2  //Define the direction control pin of the right motor
#define MR_PWM 5   //Define the PWM control pin of the right motor
#define Trig 12
#define Echo 13
float distance;
Servo myservo;

void setup() {
  Serial.begin(9600);
  myservo.attach(10);
  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);
  pinMode(ML_Ctrl, OUTPUT);
  pinMode(ML_PWM, OUTPUT);
  pinMode(MR_Ctrl, OUTPUT);
  pinMode(MR_PWM, OUTPUT);
  procedure(90); //Set the angle of the servo to 90°
  delay(500); //delay in 500ms
}

void loop() {
  a = checkdistance("initial");  //Assign the distance to the front detected by ultrasonic sensor to the variable a

  if (a < 20) {//When the distance to the front is less than 20cm
    Car_Stop();  //The robot stops
    delay(500); //delay in 500ms
    myservo.write(180);  //Ultrasonic pan-tilt turns left
    delay(1500); //delay in 500ms
    a1 = checkdistance("a1");  //Assign the distance to the left detected by ultrasonic sensor to the variable a1
    delay(100); //read value
    myservo.write(0);
    delay(1500); //delay in 500ms
    a2 = checkdistance("a2"); //Assign the distance to the right detected by ultrasonic sensor to the variable a2
    delay(100); //read value
    
    myservo.write(90);
    delay(500);
    if (a1 > a2 & a1 + a2 > 40) { //When the distance to the left is bigger than to the right
      Car_left();  //The robot turns left
      delay(700);  //turn left700ms
    } else if (a1 < a2 & a1 + a2 > 40) {
      Car_right(); //It turns left for 700ms
      delay(700);
    } else {
      Car_back();
      delay(1700);
    }
  } 
  else { //When the distance to the front is >=20c，the robot moves forward
    Car_front(); //go front
  }
}

void Car_front() {
  digitalWrite(MR_Ctrl, HIGH);
  analogWrite(MR_PWM, 650);
  digitalWrite(ML_Ctrl, HIGH);
  analogWrite(ML_PWM, 660);
}

void Car_back() {
  digitalWrite(MR_Ctrl, LOW);
  analogWrite(MR_PWM, 100);
  digitalWrite(ML_Ctrl, LOW);
  analogWrite(ML_PWM, 100);
}

void Car_left() {
  delay(100);
  Car_back();
  delay(500);
  digitalWrite(MR_Ctrl, HIGH);
  analogWrite(MR_PWM, 110);
  digitalWrite(ML_Ctrl, LOW);
  analogWrite(ML_PWM, 360);
}

void Car_right() {
  delay(100);
  Car_back();
  delay(500);
  digitalWrite(MR_Ctrl, LOW);
  analogWrite(MR_PWM, 360);
  digitalWrite(ML_Ctrl, HIGH);
  analogWrite(ML_PWM, 110);
}

void Car_Stop() {
  digitalWrite(MR_Ctrl, LOW);
  analogWrite(MR_PWM, 0);
  digitalWrite(ML_Ctrl, LOW);
  analogWrite(ML_PWM, 0);
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
