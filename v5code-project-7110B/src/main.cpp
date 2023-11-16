/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                 /                                 */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----s
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"
#include "cmath"

using namespace vex;

// A global instance of competition
competition Competition;
vex::brain       Brain;


// define your global instances of motors and other devices here
controller Controller1 = controller(primary);
motor FrontLeft = motor(PORT11, ratio18_1, false);
motor FrontRight = motor(PORT1, ratio18_1, true);
motor BackTopLeft = motor(PORT19, ratio18_1, true);
motor BackTopRight = motor(PORT9, ratio18_1, false);
motor BackBottomLeft = motor(PORT20, ratio18_1, false);
motor BackBottomRight = motor(PORT8, ratio18_1, true);
pneumatics Wings = pneumatics(Brain.ThreeWirePort.A);
motor Cata = motor(PORT18, ratio18_1, true);
rotation RotationSensor = rotation(PORT17);
motor_group LeftMotors = motor_group(FrontLeft, BackBottomLeft, BackTopLeft);
motor_group RightMotors = motor_group(FrontRight, BackBottomRight, BackTopRight);
motor_group fullDrive = motor_group(FrontLeft, BackBottomLeft, BackTopLeft, FrontRight, BackBottomRight, BackTopRight);
drivetrain Drivetrain = drivetrain(LeftMotors, RightMotors);

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
  fullDrive.setStopping(coast);
  Cata.setStopping(coast);
  Brain.Screen.print("READY");
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/
void toggleWings() {
  Wings.set(!Wings.value());
}

void autonomous(void) {
  fullDrive.setVelocity(75, pct);
  fullDrive.spinFor(reverse, 600, deg);
  fullDrive.setVelocity(40, pct);
  fullDrive.spinFor(forward, 300, deg);
  fullDrive.spinFor(reverse, 400, deg);
  fullDrive.spinFor(forward, 800, deg);
  // LeftMotors.spinFor(reverse, 175, deg);
  // RightMotors.spinFor(forward, 180, deg);
  // fullDrive.spinFor(forward, 950, deg);


}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void rotateCata() {
  Cata.spinFor(forward, 365, degrees);
}

const int DEADZONE = 5;
const double TURN_IMPORTANCE = 0.35;

void usercontrol(void) {
  while (1) {
    Controller1.ButtonY.pressed(toggleWings);
    double turnVal = -(Controller1.Axis3.position(percent));
    double forwardVal = -Controller1.Axis1.position(percent);  

    // deadzone check
    if(std::abs(turnVal) < DEADZONE) {
      turnVal = 0;
    }

    if(std::abs(forwardVal) < DEADZONE) {
      forwardVal = 0;
    }

    // convert to volts
    double turnVolts = turnVal * 0.12;
    double forwardVolts = forwardVal * 0.12 * (1 - (std::abs(turnVolts) / 12.0) * TURN_IMPORTANCE);

    // arcade drive controls
    LeftMotors.spin(forward, forwardVolts + turnVolts, volt);
    RightMotors.spin(reverse, forwardVolts - turnVolts, volt);

   //cata control
    if(Controller1.ButtonR1.pressing()) {
      Cata.spin(forward, 40, pct);
    } else{
      Cata.stop(coast);
    }

    wait(20, msec);
  }
}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
