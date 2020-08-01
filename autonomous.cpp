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
 std::vector<int> a = {};
 std::vector<int> b = {};
 std::vector<int> c = {};
 std::vector<int> d = {};
  bool red=true;
  bool front =false;
  bool skills=false;
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
  4.0,
  4.0,
  4.0,
  chassis
);
void autonomous() {
  if(skills){
    int tick = 0;{
      while(tick<a.size()){
        pros::Motor(1).move(a[tick]);
        pros::Motor(10).move(b[tick]);
        pros::Motor(2).move(a[tick]);
        pros::Motor(9).move(b[tick]);
        pros::Motor(4).move(-c[tick]);
        pros::Motor(5).move(c[tick]);
        pros::Motor(3).move(d[tick]);
        pros::Motor(11).move(-d[tick]);
        tick++;
        pros::delay(10);
      }
    }
  }
  else{
    int c = 2*red-1;
    if(front){
        intake.move(126);
        intake2.move(-126);
        profileController2.generatePath({
          Point{0_ft, 0_ft, 0_deg},
          Point{2.5_ft, 0_ft, 0_deg}},
          "intake1"
        );
        printf("g");
      profileController2.setTarget("intake1");
        profileController2.waitUntilSettled();
      profileController2.setTarget("intake1",true);
        profileController2.waitUntilSettled();

        profileController2.generatePath({
          Point{0_ft, 0_ft, 0_deg},
          Point{0.1_ft, 0_ft, 0_deg}},
          "forward"
        );
      profileController2.setTarget("forward");
        profileController2.waitUntilSettled();
      chassis.turnAngle(c*-140);
      profileController2.generatePath({
        Point{0_ft, 0_ft, 0_deg},
        Point{1.1_ft, 0_ft, 0_deg}},
        "forward2"
      );
    profileController2.setTarget("forward2");
      profileController2.waitUntilSettled();
      intake.move(0);
      intake2.move(0);
      IntegratedEncoder enc = IntegratedEncoder(pros::Motor(3));//outtake
      enc.reset();

      intake.move(0);
      intake2.move(0);
      while(enc.get()<=5600){
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
    }
    else{
      chassis.setMaxVelocity(200);
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
        Point{0_ft, c*-0.90_ft, 0_deg},
        Point{2.5_ft, 0_ft, 0_deg}},
        "back1" // Profile name
      );
    profileController2.setTarget("back1",true);

    profileController1.generatePath({
      Point{0_ft, 0_ft, 0_deg},
      Point{2.7_ft, 0_ft, 0_deg}},
      "intake2" // Profile name
    );
    profileController1.setTarget("intake2");
    profileController1.waitUntilSettled();
    //stopintake
    //turn 135 degrees
    chassis.setMaxVelocity(60);
    chassis.turnAngle(c*204);
    pros::delay(400);
     
    chassis.setMaxVelocity(200);
    profileController2.generatePath({
      Point{0_ft, 0_ft, 0_deg},
      Point{2.0_ft, 0_ft, 0_deg}},
      "scorezone" // Profile name
    );
    profileController2.setTarget("scorezone");
   
    profileController2.waitUntilSettled();
     
    intake.move(0);
    intake2.move(-0);
    pros::Motor(1).move(0);
    pros::Motor(2).move(0);
    pros::Motor(10).move(0);
    pros::Motor(9).move(0);
    IntegratedEncoder enc = IntegratedEncoder(pros::Motor(3));//outtake
    enc.reset();

    intake.move(0);
    intake2.move(0);
    while(enc.get()<=5600){
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
  }
}
}
