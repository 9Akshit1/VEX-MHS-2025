#include "main.h"
#include "globals.hpp"
#include "posTracking.hpp"

// Odometry task handle
pros::Task* odomTask = nullptr;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Initializing...");

    // Calibrate IMU (important for both odometry and general use)
    pros::lcd::set_text(2, "Calibrating IMU...");
    inertial.reset();
    
    uint32_t startTime = pros::millis();
    while (inertial.is_calibrating()) {
        pros::delay(10);
    }
    uint32_t calibTime = pros::millis() - startTime;
    
    pros::lcd::set_text(2, "IMU calibrated (" + std::to_string(calibTime) + "ms)");

    // Initialize odometry if enabled
    if (ODOMETRY_ENABLED) {
        verticalEncoder1.reset();
        horizontalEncoder.reset();
        
        globalX = 0;
        globalY = 0;
        globalAngle = 0;

        // Start odometry task in background
        odomTask = new pros::Task(odometryTask, nullptr, "Odometry");
        pros::lcd::set_text(3, "Odometry: ENABLED");
    } else {
        pros::lcd::set_text(3, "Odometry: DISABLED");
    }

    pros::lcd::set_text(4, "Ready!");
    pros::delay(500);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
    // Stop all motors when disabled
    leftFront.move(0);
    leftBack.move(0);
    rightFront.move(0);
    rightBack.move(0);
    leftIntake.move(0);
    rightIntake.move(0);
    indexer.move(0);
    shooter.move(0);
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
    // Add autonomous selector or other competition-specific setup here
}

// Note: autonomous() is now in autonomous.cpp
// The linker will find it automatically
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
//void autonomous() {
    // Your autonomous code here
    // If using odometry, you can access globalX, globalY, globalAngle
    // Remember to use odom_mutex.take() and odom_mutex.give() for thread safety
    
//    pros::lcd::set_text(1, "Running Autonomous...");
//}

// Note: opcontrol() is now in opcontrol.cpp
// The linker will find it automatically
// /**
//  * Runs the operator control code. This function will be started in its own task
//  * with the default priority and stack size whenever the robot is enabled via
//  * the Field Management System or the VEX Competition Switch in the operator
//  * control mode.
//  *
//  * If no competition control is connected, this function will run immediately
//  * following initialize().
//  *
//  * If the robot is disabled or communications is lost, the
//  * operator control task will be stopped. Re-enabling the robot will restart the
//  * task, not resume it from where it left off.
//  */
// void opcontrol() {
//     pros::Controller master(pros::E_CONTROLLER_MASTER);
//     pros::Motor left_mtr(1);
//     pros::Motor right_mtr(2);
//
//     while (true) {
//         pros::lcd::print(0, "%d %d %d", (pros::lcd::read_buttons() & LCD_BTN_LEFT) >> 2,
//                          (pros::lcd::read_buttons() & LCD_BTN_CENTER) >> 1,
//                          (pros::lcd::read_buttons() & LCD_BTN_RIGHT) >> 0);
//         int left = master.get_analog(ANALOG_LEFT_Y);
//         int right = master.get_analog(ANALOG_RIGHT_Y);
//
//         left_mtr = left;
//         right_mtr = right;
//         pros::delay(20);
//     }
// }