/**
 * Kitt
 * 
 * Balance bot based on a kit
 **/


/************************************************
 * INCLUDES
 ***********************************************/
#include <Arduino.h>

#include <SparkFun_TB6612.h>
#include <MPU6050.h>
#include <I2Cdev.h>

#include "Wire.h"
#include "math.h"


/************************************************
 * DEFINES
 ***********************************************/
//Motor pin defines
#define MOTOR_A_PWM    10
#define MOTOR_A_IN1    13
#define MOTOR_A_IN2    12
#define MOTOR_A_OFFSET 1

#define MOTOR_B_PWM    9
#define MOTOR_B_IN1    7
#define MOTOR_B_IN2    8
#define MOTOR_B_OFFSET -1

//PID parameters
#define Kp  500
#define Ki  250
#define Kd  150
#define sampleTime  0.005
#define targetAngle 0


/************************************************
 * CLASS INITIALISATION
 ***********************************************/
Motor motor1 = Motor(MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_A_PWM, MOTOR_A_OFFSET);
Motor motor2 = Motor(MOTOR_B_IN1, MOTOR_B_IN2, MOTOR_B_PWM, MOTOR_B_OFFSET);

MPU6050 mpu;


/************************************************
 * GLOBAL VARIABLES
 ***********************************************/
int16_t accY, accZ, gyroX;
volatile int motorPower, gyroRate;
volatile float accAngle, gyroAngle, currentAngle, prevAngle = 0, error, prevError = 0, errorSum = 0;
volatile byte count = 0;


/************************************************
 * FUNCTIONS  
 ***********************************************/
void initPID() {
  // init Timer1
  cli();                  // Disable global interrupts
  TCCR1A = 0;             // Set entire TCCR1A register to 0
  TCCR1B = 0;             // Set entire TCCR1B register to 0
  OCR1A = 9999;           // Set compare match register to set sample time 5ms
  TCCR1B |= 1 << CS11;    // Set CS11 bit for prescaling by 8
  TIMSK1 |= 1 << OCIE1A;  // Enable timer compare interrupt
  sei();                  // Enable global interrupts
}


/************************************************
 * MAIN PROGRAM
 ***********************************************/
void setup() {
  mpu.initialize();

  // Adjust MPU6050 offsets
  mpu.setXAccelOffset(-1065);
  mpu.setYAccelOffset(1137);
  mpu.setZAccelOffset(1240);
  mpu.setXGyroOffset(58);
  mpu.setYGyroOffset(-9);
  mpu.setZGyroOffset(40);

  // Init PID sampling loop
  initPID();
}

void loop() { 
  // Read acceleration and gyroscope values
  accY = mpu.getAccelerationY();
  accZ = mpu.getAccelerationZ();
  gyroX = mpu.getRotationX();

  // Set motor power after constraining it
  motorPower = constrain(motorPower, -255, 255);
  forward(motor1, motor2, (int)motorPower);
}

/************************************************
 * INTERRUPT SERVICE ROUTINE
 ***********************************************/
ISR(TIMER1_COMPA_vect) {
  // The ISR will be called every 5 milliseconds
  // Calculate the angle of inclination
  accAngle = atan2(accY, accZ) * RAD_TO_DEG;
  gyroRate = map(gyroX, -32768, 32767, -250, 250);
  gyroAngle = (float)gyroRate * sampleTime;
  currentAngle = 0.9934 * (prevAngle + gyroAngle) + 0.0066 * accAngle;

  // Calculate the error
  error = currentAngle - targetAngle;
  errorSum = errorSum + error;
  errorSum = constrain(errorSum, -300, 300);

  // Calculate the output from P, I and D values
  motorPower = Kp * error + Ki * error * sampleTime + Kd * (currentAngle - prevAngle)/sampleTime;
  prevAngle = currentAngle;
}