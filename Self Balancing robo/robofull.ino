#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

// Motor A
int motorA1 = 5;  
int motorA2 = 6;  
int enableA = 9;  

// Motor B
int motorB1 = 8;   
int motorB2 = 7;   
int enableB = 10;  

// Variables to hold motor speed
int speed = 0;

// Angle where robot is balanced
float setpoint = -3.40;  

// PID constants
double Kp = 100;  
double Ki = 0;   
double Kd = 0.5;  

// PID variables
double error, lastError;
double integral, derivative;
double output;

// Complementary filter variables
float angle = 0.0;
float alpha = 0.98;
unsigned long lastTime;

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing...");
  Wire.begin();
  mpu.initialize();

  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed");
    while (1) {
      delay(1000);  
      Serial.println("Attempting to reconnect...");
    }
  } else {
    Serial.println("MPU6050 connection successful");
  }

  pinMode(motorA1, OUTPUT);
  pinMode(motorA2, OUTPUT);
  pinMode(enableA, OUTPUT);

  pinMode(motorB1, OUTPUT);
  pinMode(motorB2, OUTPUT);
  pinMode(enableB, OUTPUT);

  lastTime = millis();
}

void loop() {

  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);


  float accAngle = atan2(ay, az) * 180 / PI;

  
  float gyroRate = gx / 131.0;
  unsigned long currentTime = millis();
  float dt = (currentTime - lastTime) / 1000.0;
  lastTime = currentTime;


  angle = alpha * (angle + gyroRate * dt) + (1 - alpha) * accAngle;

 
  error = angle - setpoint;

  integral += error * dt;
  

  if (integral > 100) integral = 100;
  if (integral < -100) integral = -100;

  derivative = (error - lastError) / dt;
  lastError = error;
  output = Kp * error + Ki * integral + Kd * derivative;

  speed = output;


  if (speed > 255) speed = 255;
  if (speed < -255) speed = -255;


  if (abs(error) < 0.5) {  
    speed = 0;
  }


  if (speed > 0) {
    analogWrite(enableA, speed);
    digitalWrite(motorA1, HIGH);
    digitalWrite(motorA2, LOW);
    analogWrite(enableB, speed);
    digitalWrite(motorB1, HIGH);
    digitalWrite(motorB2, LOW);
  } else if (speed < 0) {
    analogWrite(enableA, -speed);
    digitalWrite(motorA1, LOW);
    digitalWrite(motorA2, HIGH);
    analogWrite(enableB, -speed);
    digitalWrite(motorB1, LOW);
    digitalWrite(motorB2, HIGH);
  } else {
    analogWrite(enableA, 0);
    digitalWrite(motorA1, LOW);
    digitalWrite(motorA2, LOW);
    analogWrite(enableB, 0);
    digitalWrite(motorB1, LOW);
    digitalWrite(motorB2, LOW);
  }

  delay(10);  
}
