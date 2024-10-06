#include "Servo.h"
#include "Wire.h"       
#include "I2Cdev.h"     
#include "MPU6050.h"
#define servoPin 10  //The pin of servo
#define leftMotorCtrl 4  //Define the direction control pin of the left motor
#define leftMotorPwr 6   //Define the PWM control pin of the left motor
#define rightMotorCtrl 2  //Define the direction control pin of the right motor
#define rightMotorPwr 5   //Define the PWM control pin of the right motor
#define trig 12           //ultrasonic trigger
#define echo 13           //ultrasonic sensor
#define leftLineTrk 11  //left line traking sensor
#define midLineTrk 7  //middle line traking sensor
#define rightLineTrk 8  //right line traking sensor
#define soundPin 9
float distance;
int a, a1, a2;
int leftLineTrkVal, midLineTrkVal, rightLineTrkVal;
Servo myservo;

MPU6050 mpu; //i2c gyro address 0x68
int16_t ax, ay, az;
int16_t gx, gy, gz;

struct MyData {
  byte ax, ay, az, gx, gy, gz;
};

MyData data;

void setup() {
  Serial.begin(9600);
  Wire.begin();
  mpu.initialize();
  myservo.attach(10);
  myservo.write(90);
  pinMode(trig, OUTPUT);
  pinMode(soundPin, OUTPUT);
  pinMode(echo, INPUT);
  pinMode(leftMotorCtrl, OUTPUT);
  pinMode(leftMotorPwr, OUTPUT);
  pinMode(rightMotorCtrl, OUTPUT);
  pinMode(rightMotorPwr, OUTPUT);
  pinMode(leftLineTrk, INPUT); //Set all pins of the line tracking sensor as input mode
  pinMode(midLineTrk, INPUT);
  pinMode(rightLineTrk, INPUT);
  delay(500); //delay in 500ms
}

void loop() {
  a = checkdistance("initial");  //Assign the distance to the front detected by ultrasonic sensor to the variable a

  lineTrakingData();
  delay(500);
  Serial.print("!isFallBack = ");
  Serial.println(!isFallBack);

  if (!isFallBack() && !isStuck()) {
    helpSignal(false);
    if (a < 20) {//When the distance to the front is less than 20cm
      carStop();  //The robot stops
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
        carLeft();  //The robot turns left
        delay(700);  //turn left700ms
      } else if (a1 < a2 ) {
        carRight(); //It turns left for 700ms
        delay(700);
      } else {
        //carBack();
        delay(1700);
      }
    } 
    else { //When the distance to the front is >=20c，the robot moves forward
      carFront(); //go front
    }
  } else {
    carStop();
    helpSignal(true);
    delay(5000);
  }

}

//Axis X = 132  Axis Y = 128  Axis Z  = 246 -> normal
//Axis X = 138  Axis Y = 245  Axis Z  = 99 -> fall
bool isFallBack() {
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  data.ax = map(ax, -17000, 17000, 0, 255 );
  data.ay = map(ay, -17000, 17000, 0, 255); 
  data.az = map(az, -17000, 17000, 0, 255);
  Serial.print("Axis Y = ");
  Serial.println(data.ay);
  Serial.print("Axis Z = ");
  Serial.println(data.az);
  if (data.ay > 230 & data.az < 120) {
    return true;
  }
  return false;
}

bool isStuck() {
  // Получаем данные акселерометра и гироскопа
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Преобразуем данные акселерометра в g (ускорение свободного падения)
  float accelX_g = ax / 16384.0;  // Для диапазона ±2g
  float accelY_g = ay / 16384.0;
  float accelZ_g = az / 16384.0;

  // Преобразуем данные гироскопа в градусы/секунду (для диапазона ±250°/сек)
  float gyroX_dps = gx / 131.0;
  float gyroY_dps = gy / 131.0;
  float gyroZ_dps = gz / 131.0;

  // Проверяем, двигается ли робот:
  // Если угловая скорость близка к нулю и ускорение близко к гравитации
  if (abs(gyroX_dps) < 3 && abs(gyroY_dps) < 2 && abs(gyroZ_dps) < 1) { // Робот не вращается
    if (abs(accelX_g) < 0.07 && abs(accelY_g) < 0.15 && abs(accelZ_g - 1.0) < 0.07) { // Робот не движется
      Serial.println("Робот стоит на месте.");
      return true;
    } else {
      Serial.println("Робот движется.");
      return false;
    }
  } else {
    Serial.println("Робот вращается.");
    return false;
  }

}

//The function controls ultrasonic sound
float checkdistance(String from) {
  float distance;
  digitalWrite(trig, LOW);
  delayMicroseconds(2);
  digitalWrite(trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig, LOW);
  distance = pulseIn(echo, HIGH) / 58.20;  //The 58.20 here comes from 2*29.1=58.2
  Serial.print("distance " + from + ": ");
  Serial.print(distance);
  Serial.println();
  delay(10);
  return distance;
}

void helpSignal(bool enable) {
  if (enable) {
    for (int i = 100; i < 1000; i += 10) {
      tone(soundPin, i);
      delay(10);
    }
    for (int i = 1000; i > 100; i -= 10) {
      tone(soundPin, i);
      delay(10);
    }
  } else {
    noTone(soundPin);
  }
  
}

void carFront() {
  digitalWrite(rightMotorCtrl, HIGH);
  analogWrite(rightMotorPwr, 650);
  digitalWrite(leftMotorCtrl, HIGH);
  analogWrite(leftMotorPwr, 660);
}

void carBack() {
  digitalWrite(rightMotorCtrl, LOW);
  analogWrite(rightMotorPwr, 100);
  digitalWrite(leftMotorCtrl, LOW);
  analogWrite(leftMotorPwr, 100);
}

void carLeft() {
  //delay(100);
  //carBack();
  //delay(500);
  digitalWrite(rightMotorCtrl, HIGH);
  analogWrite(rightMotorPwr, 100);
  digitalWrite(leftMotorCtrl, LOW);
  analogWrite(leftMotorPwr, 365);
}

void carRight() {
  //delay(100);
  //carBack();
  //delay(500);
  digitalWrite(rightMotorCtrl, LOW);
  analogWrite(rightMotorPwr, 365);
  digitalWrite(leftMotorCtrl, HIGH);
  analogWrite(leftMotorPwr,100);
}

void carStop() {
  digitalWrite(rightMotorCtrl, LOW);
  analogWrite(rightMotorPwr, 0);
  digitalWrite(leftMotorCtrl, LOW);
  analogWrite(leftMotorPwr, 0);
}

//Read the value of the line tracker sensors
void lineTrakingData() {
  leftLineTrkVal = digitalRead(leftLineTrk);
  Serial.print("left line trk: ");
  Serial.println(leftLineTrkVal);

  midLineTrkVal = digitalRead(midLineTrk);
  Serial.print("mid line trk: ");
  Serial.println(midLineTrkVal);

  rightLineTrkVal = digitalRead(rightLineTrk);
  Serial.print("right line trk: ");
  Serial.println(rightLineTrkVal);
}
