#include "main.h"
#include <stdio.h>
#include <string>
#include "okapi/api.hpp"
 
using namespace okapi;
 
#define LEFT_WHEELS_PORT 2
#define LEFT_WHEELS_PORT_2 6
#define RIGHT_WHEELS_PORT 5
#define RIGHT_WHEELS_PORT_2 3
#define INTAKE_PORT_L 8
#define INTAKE_PORT_R 7
#define OUTTAKE_PORT 20
#define LIFT_PORT 9
#define VISION_PORT 17
//testing something
bool profiling = false;
 
//ports/encoders
pros::Motor left_wheels (LEFT_WHEELS_PORT);
pros::Motor left_wheels_2 (LEFT_WHEELS_PORT_2,true);
pros::Motor right_wheels (RIGHT_WHEELS_PORT, true);
pros::Motor right_wheels_2 (RIGHT_WHEELS_PORT_2);
pros::Motor intake_L (INTAKE_PORT_L);
pros::Motor intake_R (INTAKE_PORT_R);
pros::Motor lift (OUTTAKE_PORT_2,MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_COUNTS);
pros::Motor outtake (OUTTAKE_PORT,MOTOR_GEARSET_36, false, pros::E_MOTOR_ENCODER_COUNTS);
IntegratedEncoder enc = IntegratedEncoder(outtake);
 
//budget motion profiling???
std::vector<int> right_motor_movement_log={};
std::vector<int> left_motor_movement_log={};
std::vector<int> outtake_motor_movement_log={};
std::vector<int> intake_motor_movement_log={};
 
//various numbers
float SPEED_COEFFICIENT=126/127;
float SPEED_FAST=126/127;
float SPEED_SLOW=0.4;
int INTAKE_SPEED=126;
int OUTTAKE_ENCODER_TICKS=5600;//???
int OUTTAKE_SPEED=126;//????
int LIFT_SPEED=126;
 
//color codes?
 
pros::Vision vis (VISION_PORT);
 
 
pros::vision_signature_s_t THANOSCUBE = pros::Vision::signature_from_utility(1, 1379, 2243, 1811, 8235, 9945, 9090, 3.000, 1);
pros::vision_signature_s_t ORANGCUBE = pros::Vision::signature_from_utility(2, 6253, 6967, 6610, -2621, -2185, -2403, 3.000, 0);
pros::vision_signature_s_t GREENCUBES = pros::Vision::signature_from_utility(3, -8371, -7575, -7973, -4897, -3719, -4308, 3.000, 0);
pros::vision_signature_s_t PURPLE = pros::Vision::signature_from_utility( 4, 895, 1927, 1411, 8041, 10169, 9105, 3.000, 1);
//motion profiling variables??
auto myChassis = ChassisControllerFactory::create(
  {1, -2}, // Left motors
  {-10, 9},   // Right motors
  AbstractMotor::gearset::red, // Torque gearset
  {4_in, 12.5_in} // 4 inch wheels, 12.5 inch wheelbase width
);
 
auto profileController = AsyncControllerFactory::motionProfile(
  2.0,  // Maximum linear velocity of the Chassis in m/s
  2.0,  // Maximum linear acceleration of the Chassis in m/s/s
  10.0, // Maximum linear jerk of the Chassis in m/s/s/s
  myChassis // Chassis Controller
);
 
int autoAlignCube(){
  pros::vision_object_s_t purp = vis.get_by_sig(0, 1);
  pros::vision_object_s_t green = vis.get_by_sig(0, 2);
  pros::vision_object_s_t orang = vis.get_by_sig(0, 3);
  int size1= purp.width*purp.height;
  int size2= green.width*green.height;
  int size3= orang.width*orang.height;
  float left=-0;
  float right=0;
  if(size1>=size2&&size1>=size3){
    int x = purp.left_coord;
    double y = -1.2307*x+128.462;
    printf("%d %d %d\n",purp.x_middle_coord,purp.y_middle_coord,y);
    if(abs(purp.top_coord-y)<10){
      printf("true");
      return 0;
    }
    else if(purp.top_coord-y<=10){
      return 30;
    }
    else{
      return -30;
    }
  }
  else if(size2>size3){
 
      int x = green.left_coord;
      double y = -1.2307*x+128.462;
      printf("%d %d %d\n",purp.x_middle_coord,purp.y_middle_coord,y);
      if(abs(green.top_coord-y)<10){
        printf("true");
        return 0;
      }
      else if(green.top_coord-y<=10){
        return 30;
      }
      else{
        return -30;
      }
    }
  else{
 
      int x = orang.left_coord;
      double y = -1.2307*x+128.462;
      printf("%d %d %d\n",purp.x_middle_coord,purp.y_middle_coord,y);
      if(abs(orang.top_coord-y)<10){
        printf("true");
        return 0;
      }
      else if(orang.top_coord-y<=10){
        return 30;
      }
      else{
        return -30;
      }
  }
}
 
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
void moveOuttake(bool dir){
 if(dir){
  outtake.move(OUTTAKE_SPEED);
  outtake_motor_movement_log.push_back(OUTTAKE_SPEED);
 }
 else{
  outtake.move(-OUTTAKE_SPEED);
  outtake_motor_movement_log.push_back(-OUTTAKE_SPEED);
 }
}
void stopOuttake(){
  outtake.move(0);
  outtake_motor_movement_log.push_back(0);
}
void outtake_macro(bool state,long encStart){//false for reverse
  printf("%d\n",enc.get());
  if(state==1){
    if(enc.get()-encStart<OUTTAKE_ENCODER_TICKS){
      outtake.move(OUTTAKE_SPEED);
      outtake_motor_movement_log.push_back(OUTTAKE_SPEED);
    }
    else{
      stopOuttake();
    }
  }
  if(state==0){
    if(enc.get()>0){
      outtake.move(-OUTTAKE_SPEED);
      outtake_motor_movement_log.push_back(-OUTTAKE_SPEED);
    }
    else{
      stopOuttake();
    }
  }
}
void outtake_macro(bool dir){//false for reverse
  IntegratedEncoder enc = IntegratedEncoder(outtake);
  enc.reset();
  if(dir){
    while(enc.get()<=OUTTAKE_ENCODER_TICKS){
      outtake.move(OUTTAKE_SPEED);
      outtake_motor_movement_log.push_back(OUTTAKE_SPEED);
    }
  }
  else{
    while(enc.get()>=-OUTTAKE_ENCODER_TICKS){
      outtake.move(-OUTTAKE_SPEED);
      outtake_motor_movement_log.push_back(-OUTTAKE_SPEED);
    }
  }
  outtake.move(0);
  outtake_motor_movement_log.push_back(-OUTTAKE_SPEED);
}
void move_lift(bool dir){
  if(dir){
    lift.move(LIFT_SPEED);
  }
  else{
    lift.move(-LIFT_SPEED);
  }
}
void stop_lift(){
  lift.move(0);
}
void opcontrol() {
  vis.set_signature(1, &THANOSCUBE);
  vis.set_signature(2, &GREENCUBES);
  vis.set_signature(3, &ORANGCUBE);
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
     //DRIVE (TANK)
     float left=(master.get_analog(ANALOG_LEFT_Y)*SPEED_COEFFICIENT);
     float right=(master.get_analog(ANALOG_RIGHT_Y)*SPEED_COEFFICIENT);
     if(master.get_digital(DIGITAL_L2)){
       int adjust = autoAlignCube();
       if(adjust==-30){
         right-=30;
       }
       if(adjust==30){
         left-=30;
       }
     }
     left_wheels.move(left);
     right_wheels.move(right);
     left_wheels_2.move(left);
     right_wheels_2.move(right);
     left_motor_movement_log.push_back(left);
     right_motor_movement_log.push_back(right);
    //DRIVE (ARCADE)
     /*int power = master.get_analog(ANALOG_LEFT_Y);
     int turn = master.get_analog(ANALOG_RIGHT_X);
     int left = (power + turn)*SPEED_COEFFICIENT;
     int right = (power - turn)*SPEED_COEFFICIENT;
     right *= -1; // This reverses the right motor
     left_wheels.move(left);
     right_wheels.move(right);
     left_wheels_2.move(left);
     right_wheels_2.move(right);
     left_motor_movement_log.push_back(left);
     right_motor_movement_log.push_back(right);*/
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
    if(master.get_digital(DIGITAL_X)){
      //outtake_macro(true);
      moveOuttake(true);//controlled outtake
    }
    else if(master.get_digital(DIGITAL_UP)){
      //outtake_macro(true);
      moveOuttake(false);
    }
    else{
     stopOuttake();
    }
    //LIFT SYSTEM
    if(master.get_digital(DIGITAL_X)){
      //outtake_macro(true);
      moveOuttake(true);//controlled outtake
    }
    else if(master.get_digital(DIGITAL_UP)){
      //outtake_macro(true);
      moveOuttake(false);
    }
    else{
     stopOuttake();
    }
    bool writtofile=false;
    if(master.get_digital(DIGITAL_Y)&&!writtofile){
      FILE* left = fopen("/usd/left.txt", "w");
      fputs("{", left);
      for(int i:left_motor_movement_log){
        fprintf(left,"%d, ",i);
      }
      fputs("}", left);
      fclose(left);FILE* right = fopen("/usd/right.txt", "w");
      fputs("{", right);
      for(int i:right_motor_movement_log){
        fprintf(right,"%d, ",i);
      }
      fputs("}", right);
      fclose(right);
      FILE* out = fopen("/usd/out.txt", "w");
      fputs("{", out);
      for(int i:outtake_motor_movement_log){
        fprintf(out,"%d, ",i);
      }
      fputs("}", out);
      fclose(out);
      FILE* inta = fopen("/usd/inta.txt", "w");
      fputs("{", inta);
      for(int i:intake_motor_movement_log){
        fprintf(inta,"%d, ",i);
      }
      fputs("}", inta);
      fclose(inta);
      writtofile=true;
    }
    pros::delay(10);
}
