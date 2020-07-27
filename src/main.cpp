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


/************************************************
 * DEFINES
 ***********************************************/
#define MOTOR_A_PWM    10
#define MOTOR_A_IN1    13
#define MOTOR_A_IN2    12
#define MOTOR_A_OFFSET 1

#define MOTOR_B_PWM    9
#define MOTOR_B_IN1    7
#define MOTOR_B_IN2    8
#define MOTOR_B_OFFSET -1


/************************************************
 * CLASS INITIALISATION
 ***********************************************/
Motor motor1 = Motor(MOTOR_A_IN1, MOTOR_A_IN2, MOTOR_A_PWM, MOTOR_A_OFFSET);
Motor motor2 = Motor(MOTOR_B_IN1, MOTOR_B_IN2, MOTOR_B_PWM, MOTOR_B_OFFSET);


/************************************************
 * MAIN PROGRAM
 ***********************************************/
void setup() {

}

void loop() {
  motor1.drive(255, 1000);
  motor1.drive(-255, 1000);
  motor1.brake();
  delay(1000);

  motor2.drive(255, 1000);
  motor2.drive(-255, 1000);
  motor2.brake();
  delay(1000);

  forward(motor1, motor2, 150);
  delay(1000);

  back(motor1, motor2, 150);
  delay(1000);

  brake(motor1, motor2);
  delay(1000);

  left(motor1, motor2, 100);
  delay(1000);
  right(motor1, motor2, 100);
  delay(1000);

  brake(motor1, motor2);
  delay(1000);
}