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
#define LIFT_PORT 15

int currlevel=0;

pros::Motor left_wheels (LEFT_WHEELS_PORT);
pros::Motor left_wheels_2 (LEFT_WHEELS_PORT_2,true);
pros::Motor right_wheels (RIGHT_WHEELS_PORT, true);
pros::Motor right_wheels_2 (RIGHT_WHEELS_PORT_2);
pros::Motor intake_L (INTAKE_PORT_L);
pros::Motor intake_R (INTAKE_PORT_R);
pros::Motor lift_motor (LIFT_PORT,MOTOR_GEARSET_36);
pros::Motor outtake (OUTTAKE_PORT, E_MOTOR_GEARSET_18, false, pros::E_MOTOR_ENCODER_COUNTS);

bool pres=false;

void moveIntake(bool dir){
  if(dir){
    intake_L.move(-126);
    intake_R.move(126);
  }
  else{
    intake_L.move(126);
    intake_R.move(-126);
  }
}
void moveOuttake(bool dir){
  if(dir){
    outtake.move(61);
  }
  else{
    outtake.move(-61)
  }
}
void stopIntake(){
  intake_L.move(0);
  intake_R.move(0);
}
void stopIntake(){
  outake.move(0);
}
void outtake_macro(){
  int numrot= 10;
  IntegratedEncoder enc = IntegratedEncoder(outtake);
  enc.reset();
  while(enc.get()<=1800*numrot){
    printf("tick: %d\n",enc.get());
    outtake.move(126);
  }
  outtake.move(0);
}
void lift(int level){
  double heights[5] = {0,18.83,24.66,37.91};
  double conv = 5;
  int currot = (heights[level]-heights[currlevel])/conv;
  IntegratedEncoder enc = IntegratedEncoder(outtake);
  enc.reset();
  while(enc.get()<=1800*currot){
    //printf("tick: %d\n",enc.get());
    lift_motor.move(126);
  }
  lift_motor.move(0);
}
void opcontrol() {

  pros::Controller master (CONTROLLER_MASTER);

  while (true) {
    outtake.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);    
    left_wheels.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    right_wheels.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    left_wheels_2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    right_wheels_2.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    lift_motor.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
    char const* currlevelStr =(std::to_string(currlevel)).c_str(); //doesntwork
    printf("level: %s\n",currlevelStr);//doesntwork
    master.set_text(1, 2,currlevelStr);//doesntwork
		int left=master.get_analog(ANALOG_LEFT_Y);
		int right=master.get_analog(ANALOG_RIGHT_Y);
    left_wheels.move(left);
    right_wheels.move(right);
    left_wheels_2.move(left);
    right_wheels_2.move(right);
    if (master.get_digital(DIGITAL_R1)) {
      moveIntake(true);
    }
    else if (master.get_digital(DIGITAL_R2)) {
      moveIntake(false);
    }
    else {
      stopIntake();
    }
    //printf("level: %d %d\n",currlevel,pres);
    if(master.get_digital(DIGITAL_L1)&&!pres){
      printf("level: %d %d\n",currlevel,pres);
      pres=true;
      currlevel++;
      if(currlevel==5){
        currlevel=0;
      }
      //lift(currlevel);
    }
    else if(master.get_digital(DIGITAL_L2)&&!pres){
      pres=true;
      printf("level: %d %d\n",currlevel,pres);
      currlevel--;
      if(currlevel==-1){
        currlevel=4;
      }
      //lift(currlevel);
    }
    else if(master.get_digital(DIGITAL_L1)+master.get_digital(DIGITAL_L2)==0){
      pres=false;
    }
    if(master.get_digital(DIGITAL_UP)){
      moveOuttake(true);
      //outtake_macro();
    }
    else if(master.get_digital(DIGITAL_DOWN)){
      moveOuttake(false);
    }
    else{
      stopOuttake();
    }
    pros::delay(2);
  }
}
