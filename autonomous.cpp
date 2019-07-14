#include "main.h"
#include <stdio.h>
#include "okapi/api.hpp"
 
using namespace okapi;
/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
 
 pros::Motor intake (4);
 pros::Motor intake2 (5);
 auto chassis = ChassisControllerFactory::create(
  {1, -2}, {10, -9},
  AbstractMotor::gearset::green,
  {4_in, 12.5_in}
);
auto profileController1 = AsyncControllerFactory::motionProfile(
  0.20,  // Maximum linear velocity of the Chassis in m/s
  2.0,  // Maximum linear acceleration of the Chassis in m/s/s
  10.0, // Maximum linear jerk of the Chassis in m/s/s/s
  chassis // Chassis Controller
);
 
auto profileController2= AsyncControllerFactory::motionProfile(
  6.0,
  6.0,  
  10.0, 
  chassis 
);
void autonomous() {
    intake.move(126);
    intake2.move(-126);
    profileController1.generatePath({
      Point{0_ft, 0_ft, 0_deg}, 
      Point{1.5_ft, 0_ft, 0_deg}},
      "intake1" 
    );
    printf("done");
  profileController1.setTarget("intake1");
    profileController1.waitUntilSettled();
    profileController2.generatePath({
      Point{0_ft, 0_ft, 0_deg},
      Point{1.2_ft, 0_ft, 0_deg}},
      "back1" // Profile name
    );
  profileController2.setTarget("back1",true);
  profileController2.waitUntilSettled();
  pros::delay(500);
  chassis.turnAngle(-133);
  pros::delay(50);
  profileController2.generatePath({
    Point{0_ft, 0_ft, 0_deg},
    Point{1.1_ft, 0_ft, 0_deg}},
    "tosecondpoint" // Profile name
  );
  profileController2.setTarget("tosecondpoint");
  profileController2.waitUntilSettled();
  chassis.turnAngle(133);
  pros::delay(50);
 
  profileController1.generatePath({
    Point{0_ft, 0_ft, 0_deg},
    Point{1.3_ft, 0_ft, 0_deg}},
    "intake2" // Profile name
  );
  profileController1.setTarget("intake2");
  profileController1.waitUntilSettled();
  //slightouttake
  intake.move(-40);
  intake2.move(40);
  pros::delay(40);
  //stopintake
  intake.move(0);
  intake2.move(-0);
  //turn 135 degrees
  chassis.turnAngle(210);
  pros::delay(100);
  profileController2.generatePath({
    Point{0_ft, 0_ft, 0_deg},
    Point{2.7_ft, 0_ft, 0_deg}},
    "scorezone" // Profile name
  );
  profileController2.setTarget("scorezone");
  profileController2.waitUntilSettled();
  IntegratedEncoder enc = IntegratedEncoder(pros::Motor(3));//outtake
  enc.reset();
 
  while(enc.get()<=5500){
    pros::Motor(3).move(126);
    pros::Motor(11).move(-126);
  }
  pros::Motor(3).move(0);
  pros::Motor(11).move(0);
  //backoff
  pros::Motor(1).move(-50);
  pros::Motor(2).move(-50);
  pros::Motor(10).move(-50);
  pros::Motor(9).move(-50);
  pros::delay(100);
  pros::Motor(10).move(0);
  pros::Motor(9).move(0);
}
