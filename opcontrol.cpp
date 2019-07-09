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
 
//testing something
bool profiling = false;
 
//ports/encoders
pros::Motor left_wheels (LEFT_WHEELS_PORT);
pros::Motor left_wheels_2 (LEFT_WHEELS_PORT_2,true);
pros::Motor right_wheels (RIGHT_WHEELS_PORT, true);
pros::Motor right_wheels_2 (RIGHT_WHEELS_PORT_2);
pros::Motor intake_L (INTAKE_PORT_L);
pros::Motor intake_R (INTAKE_PORT_R);
pros::Motor outtake_2 (OUTTAKE_PORT_2,MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor outtake (OUTTAKE_PORT,MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_COUNTS);
IntegratedEncoder enc = new IntegratedEncode(outtake);
 
//budget motion profiling???
std::vector<int> right_motor_movement_log={};
std::vector<int> left_motor_movement_log={};
std::vector<int> outtake_motor_movement_log={};
std::vector<int> intake_motor_movement_log={};
 
//various numbers
float SPEED_COEFFICIENT=126.0/127;
float SPEED_FAST=126.0/127;
float SPEED_SLOW=0.4;
int INTAKE_SPEED=126;
int OUTTAKE_ENCODER_TICKS=5500;//???
int OUTTAKE_SPEED=126;//????

//motion profiling variables??
auto myChassis = ChassisControllerFactory::create(
  {1, -2}, // Left motors
  {-10, 9},   // Right motors
  AbstractMotor::gearset::red, // Torque gearset
  {4_in, 12.5_in} // 4 inch wheels, 12.5 inch wheelbase width
);

auto profileController = AsyncControllerFactory::motionProfile(
  3.0,  // Maximum linear velocity of the Chassis in m/s
  2.0,  // Maximum linear acceleration of the Chassis in m/s/s
  10.0, // Maximum linear jerk of the Chassis in m/s/s/s
  myChassis // Chassis Controller
);

void moveIntake(bool dir){//false for reverse
  if(dir){
    intake_L.move(-INTAKE_SPEED);
    intake_R.move(INTAKE_SPEED);
    intake_motor_movement_log.push_back(INTAKE_SPEED);
  }
  else{
    intake_L.move(INTAKE_SPEED);
    intake_R.move(-INTAKE_SPEED);
    intake_motor_movement_log.push_back(-INTAKE_SPEED);
  }
}
void stopIntake(){
  intake_L.move(0);
  intake_R.move(0);
  intake_motor_movement_log.push_back(0);
}
void stopOuttake(){
  outtake.move(0);
  outtake_2.move(0);
  outtake_motor_movement_log.push_back(0);
}
boolean outtake_macro(boolean state){//false for reverse
  if(state==1){
    if(enc.get()<OUTTAKE_ENCODER_TICKS){
      outtake.move(OUTTAKE_SPEED);
      outtake_2.move(-OUTTAKE_SPEED);
      outtake_motor_movement_log.push_back(OUTTAKE_SPEED);
    }
    else{
      stopOuttake();
    }
  }
  if(state==0){
    if(enc.get()>0){
      outtake.move(-OUTTAKE_SPEED);
      outtake_2.move(OUTTAKE_SPEED);
      outtake_motor_movement_log.push_back(-OUTTAKE_SPEED);
    }
    else{
      stopOuttake();
    }
  }
}
void opcontrol() {
  int tick = 0;
  enc.reset();
  pros::Controller master (CONTROLLER_MASTER);
 
  outtake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  intake_R.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  intake_L.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  left_wheels.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  right_wheels.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  left_wheels_2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  right_wheels_2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
  while (true) {
    if(profiling){}
    else{
     //DRIVE (TANK)
     /*float left=(master.get_analog(ANALOG_LEFT_Y)*SPEED_COEFFICIENT);
     float right=(master.get_analog(ANALOG_RIGHT_Y)*SPEED_COEFFICIENT);
     left_wheels.move(left);
     right_wheels.move(right);
     left_wheels_2.move(left);
     right_wheels_2.move(right);
     left_motor_movement_log.push_back(left);
     right_motor_movement_log.push_back(right);
     */
    //DRIVE (ARCADE)
     int power = controller_get_analog(CONTROLLER_MASTER, ANALOG_LEFT_Y);
     int turn = controller_get_analog(CONTROLLER_MASTER, ANALOG_RIGHT_X);
     int left = (power + turn)*SPEED_COEFFICIENT;
     int right = (power - turn)*SPEED_COEFFICIENT;
     right *= -1; // This reverses the right motor
     motor_move(LEFT_WHEELS_PORT, left);
     motor_move(RIGHT_WHEELS_PORT, right);
     left_motor_movement_log.push_back(left);
     right_motor_movement_log.push_back(right);
     //SLOW MODE CONTROL
     if (master.get_digital(DIGITAL_L1)) {
       SPEED_COEFFICIENT=SPEED_SLOW;
     }
     else  {
       SPEED_COEFFICIENT=SPEED_FAST;//i hear that 126 > 127
     }
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
    bool state = false;
    if(master.get_digital(DIGITAL_X)){
      //moveOuttake(true);//controlled outtake
      state=1;
    }
    else if(master.get_digital(DIGITAL_UP)){
      //moveOuttake(false);
      state=0;
    }
    outtake_macro(state);
    if(master.get_digital(DIGITAL_Y)&&tick<left_motor_movement_log.size()){
      left_wheels.move(left_motor_movement_log[tick]);
      right_wheels.move(right_motor_movement_log[tick]);
      left_wheels_2.move(left_motor_movement_log[tick]);
      right_wheels_2.move(right_motor_movement_log[tick]);
      intake_L.move(-intake_motor_movement_log[tick]);
      intake_R.move(intake_motor_movement_log[tick]);
      outtake.move(outtake_motor_movement_log[tick]);
      outtake_2.move(-outtake_motor_movement_log[tick]);
      tick++;
    }
    pros::delay(10);
  }
}
