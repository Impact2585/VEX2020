#include "main.h"
#include <stdio.h>
#include "okapi/api.hpp"

using namespace okapi;

#define LEFT_WHEELS_PORT 1
#define LEFT_WHEELS_PORT_2 2
#define RIGHT_WHEELS_PORT 10
#define RIGHT_WHEELS_PORT_2 9
#define INTAKE_PORT_L 4
#define INTAKE_PORT_R 5
#define OUTTAKE_PORT 3
#define OUTTAKE_PORT_2 11

//ports
pros::Motor left_wheels (LEFT_WHEELS_PORT);
pros::Motor left_wheels_2 (LEFT_WHEELS_PORT_2,true);
pros::Motor right_wheels (RIGHT_WHEELS_PORT, true);
pros::Motor right_wheels_2 (RIGHT_WHEELS_PORT_2);
pros::Motor intake_L (INTAKE_PORT_L);
pros::Motor intake_R (INTAKE_PORT_R);
pros::Motor outtake_2 (OUTTAKE_PORT_2,MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor outtake (OUTTAKE_PORT,MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_COUNTS);

//budget motion profiling???
std::vector<int> right_motor_movement_log={};
std::vector<int> left_motor_movement_log={};
std::vector<int> outtake_motor_movement_log={};
std::vector<int> intake_motor_movement_log={};

//various numbers
double SPEED_COEFFICIENT=126/127;
int INTAKE_SPEED=126;
int INTAKE_NEGATIVE_SPEED=61;
int OUTTAKE_ENCODER_TICKS=4650;//???
int OUTTAKE_SPEED=86;//????

void moveIntake(bool dir){//false for reverse
  if(dir){
    intake_L.move(-INTAKE_SPEED);
    intake_R.move(INTAKE_SPEED);
    intake_motor_movement_log.push_back(INTAKE_SPEED);
  }
  else{
    intake_L.move(INTAKE_NEGATIVE_SPEED);
    intake_R.move(-INTAKE_NEGATIVE_SPEED);
    intake_motor_movement_log.push_back(-INTAKE_NEGATIVE_SPEED);
  }
}
void stopIntake(){
  intake_L.move(0);
  intake_R.move(0);
  intake_motor_movement_log.push_back(0);
}
void stopOuttake(){
  outtake.move(0);
  outtake_motor_movement_log.push_back(0);
}
void outtake_macro(bool dir){//false for reverse
  IntegratedEncoder enc = IntegratedEncoder(outtake);
  enc.reset();
  if(dir){
    while(enc.get()<=OUTTAKE_ENCODER_TICKS){
      outtake.move(OUTTAKE_SPEED);
      outtake_2.move(-OUTTAKE_SPEED);
      outtake_motor_movement_log.push_back(OUTTAKE_SPEED);
    }
  }
  else{
    while(enc.get()>=-OUTTAKE_ENCODER_TICKS){
      outtake.move(-OUTTAKE_SPEED);
      outtake_2.move(OUTTAKE_SPEED);
      outtake_motor_movement_log.push_back(-OUTTAKE_SPEED);
    }
  }
  outtake.move(0);
}
void opcontrol() {
  pros::Controller master (CONTROLLER_MASTER);

  outtake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  intake_R.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  intake_L.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  left_wheels.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  right_wheels.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  left_wheels_2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  right_wheels_2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  while (true) {
    //DRIVE (TANK)
    int left=(int)(master.get_analog(ANALOG_LEFT_Y)*SPEED_COEFFICIENT);
    int right=(int)(master.get_analog(ANALOG_RIGHT_Y)*SPEED_COEFFICIENT);
    left_wheels.move(left);
    right_wheels.move(right);
    left_wheels_2.move(left);
    right_wheels_2.move(right);
    left_motor_movement_log.push_back(left);
    right_motor_movement_log.push_back(right);
    //SLOW MODE CONTROL
    if (master.get_digital(DIGITAL_A)&&SPEED_COEFFICIENT==126/127) {
      SPEED_COEFFICIENT=60/127;
    }
    else if (master.get_digital(DIGITAL_A)) {
      SPEED_COEFFICIENT=126/127;//i hear that 126 > 127
    }
    //INTAKE CONTROL
    if (master.get_digital(DIGITAL_R1)) {
      moveIntake(true);
    }
    else if (master.get_digital(DIGITAL_R2)) {
      moveIntake(false);
    }
    else {
      stopIntake();
    }
    //OUTTAKE SYSTEM
    IntegratedEncoder enc = IntegratedEncoder(outtake);
    if(master.get_digital(DIGITAL_X)){
      //moveOuttake(true);//controlled outtake
      outtake_macro(true);
    }
    else if(master.get_digital(DIGITAL_UP)){
      //moveOuttake(false);
      outtake_macro(false);
    }
    else{
      stopOuttake();
    }
    pros::delay(5);
  }
}
