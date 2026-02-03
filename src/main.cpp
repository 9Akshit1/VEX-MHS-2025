/**
 * main.cpp - VEX Robot Control System
 * 
 * Robot Configuration:
 * - 6-motor drivetrain (all BLUE cartridges - 600 RPM)
 *   - Left: Ports 15, 16, 19 (reversed)
 *   - Right: Ports 1, 2, 3
 * - Complex intake/elevator system (4 motors - all GREEN cartridges - 200 RPM)
 *   - frontTopMotor: Port 10
 *   - frontMiddleMotor: Port 6
 *   - backTopMotor: Port 9
 *   - backMiddleMotor: Port 8
 * - Pneumatics:
 *   - Scooper: Port D (controlled by X/B)
 *   - Piston2: Port A (controlled by A/Y)
 * - Sensors:
 *   - Vertical Encoder: 3-wire ports B,C
 *   - Horizontal Encoder: 3-wire ports H,F
 *   - IMU: Smart port (needs to be specified - assuming Port 11)
 * 
 * Control Scheme:
 * - Left Stick Y (Axis3): Forward/Backward
 * - Left Stick X (Axis1): Turning
 * - R1: Intake middle (Port 6 forward, others run to support)
 * - R2: Intake top (shoot to top - all motors forward)
 * - L1: Intake to bottom storage
 * - L2: Intake to top storage
 * - D-Pad Up: Port 6 CCW
 * - D-Pad Down: Port 9 CCW
 * - D-Pad Left: Port 8 CW
 * - D-Pad Right: Port 10 CW
 * - X: Extend scooper (Port D)
 * - B: Retract scooper (Port D)
 * - A: Extend piston2 (Port A)
 * - Y: Retract piston2 (Port A)
 */

#include "main.h"
#include "lemlib/api.hpp"
#include <cmath>

// ==================== MOTOR DECLARATIONS ====================

// DRIVETRAIN - 6 motors (3 per side, all BLUE cartridges - 600 RPM)
pros::Motor leftFront(-15, pros::MotorGearset::blue);      // Port 15 (reversed)
pros::Motor leftMiddle(-16, pros::MotorGearset::blue);     // Port 16 (reversed)
pros::Motor leftBack(-19, pros::MotorGearset::blue);       // Port 19 (reversed)

pros::Motor rightFront(1, pros::MotorGearset::blue);       // Port 1
pros::Motor rightMiddle(2, pros::MotorGearset::blue);      // Port 2
pros::Motor rightBack(3, pros::MotorGearset::blue);        // Port 3

// INTAKE/ELEVATOR SYSTEM - 4 motors (all GREEN cartridges - 200 RPM)
pros::Motor frontTopMotor(10, pros::MotorGearset::green);     // Port 10
pros::Motor frontMiddleMotor(6, pros::MotorGearset::green);   // Port 6
pros::Motor backTopMotor(9, pros::MotorGearset::green);       // Port 9
pros::Motor backMiddleMotor(8, pros::MotorGearset::green);    // Port 8

// ==================== PNEUMATICS ====================
pros::adi::DigitalOut scooper('D');  // Port D - Scooper control (X/B buttons)
pros::adi::DigitalOut piston2('A');  // Port A - Piston2 control (A/Y buttons)

// ==================== SENSORS ====================
// Encoders (3-wire ADI ports)
pros::adi::Encoder verticalEncoder('B', 'C', false);      // Vertical tracking wheel on ports B,C
pros::adi::Encoder horizontalEncoder('H', 'F', false);    // Horizontal tracking wheel on ports H,F

// IMU (Smart port) - ADJUST PORT NUMBER AS NEEDED
pros::Imu imu(11);  // Assuming IMU is on port 11 - change if different

// ==================== CONTROLLER ====================
pros::Controller master(pros::E_CONTROLLER_MASTER);

// ==================== ROBOT CONFIGURATION ====================

// Tracking wheel specifications
const double TRACKING_WHEEL_DIAMETER = 2.75;  // inches - MEASURE YOUR ACTUAL WHEELS
const double TRACKING_WHEEL_DISTANCE = 360.0 / (TRACKING_WHEEL_DIAMETER * M_PI);  // ticks per inch

// Robot physical dimensions
const double WHEEL_DIAMETER = 3.25;           // inches (omni wheels)
const double TRACK_WIDTH = 12.0;              // inches (distance between left and right wheels)
const double WHEELBASE_LENGTH = 12.0;         // inches (front to back distance)

// Tracking wheel offsets from center of rotation - MEASURE THESE CAREFULLY
const double VERTICAL_OFFSET = 2.0;           // inches forward from center 
const double HORIZONTAL_OFFSET = 5.5;         // inches to the side from center

// Motor speeds (RPM) - All intake motors are GREEN (200 RPM max)
const int INTAKE_SPEED = 180;      // RPM for intake operations
const int OUTTAKE_SPEED = 140;     // RPM for outtake operations

// Deadzone for joysticks
const int JOYSTICK_DEADZONE = 10;

// ==================== LEMLIB SETUP ====================

// Tracking wheels
lemlib::TrackingWheel vertical(
    &verticalEncoder,
    TRACKING_WHEEL_DIAMETER,
    VERTICAL_OFFSET,
    1  // No gearing
);

lemlib::TrackingWheel horizontal(
    &horizontalEncoder,
    TRACKING_WHEEL_DIAMETER,
    HORIZONTAL_OFFSET,
    1  // No gearing
);

// PID Controllers for autonomous movement
// These values may need tuning based on your robot's performance
lemlib::ControllerSettings lateralController(
    10,   // kP - proportional gain
    0,    // kI - integral gain
    55,   // kD - derivative gain
    3,    // anti-windup
    1,    // small error range (inches)
    100,  // small error timeout (ms)
    3,    // large error range (inches)
    500,  // large error timeout (ms)
    20    // slew rate
);

lemlib::ControllerSettings angularController(
    4,    // kP - turn proportional gain
    0,    // kI - turn integral gain
    40,   // kD - turn derivative gain
    3,    // anti-windup
    1,    // small error range (degrees)
    100,  // small error timeout (ms)
    3,    // large error range (degrees)
    500,  // large error timeout (ms)
    0     // slew rate
);

// Drivetrain configuration for LemLib
pros::MotorGroup leftMotors({-15, -16, -19});
pros::MotorGroup rightMotors({1, 2, 3});

lemlib::Drivetrain drivetrain(
    &leftMotors,
    &rightMotors,
    TRACK_WIDTH,
    lemlib::Omniwheel::NEW_325,  // 3.25" omni wheels
    600,  // RPM (all BLUE motors)
    2     // horizontal drift correction
);

// Odometry sensors - NOW WITH IMU FOR HEADING
lemlib::OdomSensors sensors(
    &vertical,     // vertical tracking wheel
    nullptr,       // second vertical wheel (not used)
    &horizontal,   // horizontal tracking wheel
    nullptr,       // second horizontal wheel (not used)
    &imu           // IMU for accurate heading
);

// Create the chassis
lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);

// ==================== GLOBAL STATE VARIABLES ====================
bool scooperDeployed = false;
bool piston2Deployed = false;
bool autonomousMode = false;

// ==================== UTILITY FUNCTIONS ====================

/**
 * Apply deadzone to joystick input
 */
int applyDeadzone(int value, int deadzone = JOYSTICK_DEADZONE) {
    return (abs(value) < deadzone) ? 0 : value;
}

/**
 * Constrain value between min and max
 */
int constrain(int value, int min, int max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// ==================== DRIVETRAIN CONTROL ====================

/**
 * Omni drive control (tank drive with turning)
 * 
 * @param forward Forward/backward speed (-127 to 127)
 * @param strafe Left/right strafe speed (-127 to 127)
 * @param turn Rotation speed (-127 to 127)
 */
void omniDrive(int forward, int strafe, int turn) {
    // Apply deadzone
    forward = applyDeadzone(forward);
    strafe = applyDeadzone(strafe);
    turn = applyDeadzone(turn);
    
    // Calculate left and right power
    int leftPower = forward + strafe + turn;
    int rightPower = forward - strafe - turn;
    
    // Normalize if any value exceeds motor limits
    int maxPower = std::max(abs(leftPower), abs(rightPower));
    if (maxPower > 127) {
        double scale = 127.0 / maxPower;
        leftPower *= scale;
        rightPower *= scale;
    }
    
    // Apply to motors
    leftFront.move(leftPower);
    leftMiddle.move(leftPower);
    leftBack.move(leftPower);
    
    rightFront.move(rightPower);
    rightMiddle.move(rightPower);
    rightBack.move(rightPower);
}

/**
 * Stop all drivetrain motors
 */
void stopDrive() {
    leftFront.move(0);
    leftMiddle.move(0);
    leftBack.move(0);
    rightFront.move(0);
    rightMiddle.move(0);
    rightBack.move(0);
}

/**
 * Set brake mode for all drivetrain motors
 */
void setDriveBrakeMode(pros::motor_brake_mode_e mode) {
    leftFront.set_brake_mode(mode);
    leftMiddle.set_brake_mode(mode);
    leftBack.set_brake_mode(mode);
    rightFront.set_brake_mode(mode);
    rightMiddle.set_brake_mode(mode);
    rightBack.set_brake_mode(mode);
}

// ==================== INTAKE/ELEVATOR CONTROL ====================

/**
 * Stop all intake motors
 */
void stopIntake() {
    frontTopMotor.move(0);
    frontMiddleMotor.move(0);
    backTopMotor.move(0);
    backMiddleMotor.move(0);
}

/**
 * R1: Intake middle
 * Focus on port 6 (frontMiddle) with support from other motors
 */
void intakeMiddle() {
    frontMiddleMotor.move_velocity(INTAKE_SPEED);      // Port 6: Main intake
    backMiddleMotor.move_velocity(-INTAKE_SPEED);      // Port 8: Support
    frontTopMotor.move_velocity(-INTAKE_SPEED);        // Port 10: Support
    backTopMotor.move(0);                              // Port 9: Off
}

/**
 * R2: Intake top (shoot to top)
 * All motors forward to push balls up and out the top
 */
void intakeTop() {
    frontMiddleMotor.move_velocity(INTAKE_SPEED);      // Port 6: CW
    frontTopMotor.move_velocity(INTAKE_SPEED);         // Port 10: CW
    backMiddleMotor.move_velocity(-INTAKE_SPEED);      // Port 8: CCW
    backTopMotor.move_velocity(-INTAKE_SPEED);         // Port 9: CCW
}

/**
 * L1: Intake to bottom storage
 */
void intakeBottomStorage() {
    frontMiddleMotor.move_velocity(INTAKE_SPEED);      // Port 6: CW
    backMiddleMotor.move_velocity(INTAKE_SPEED);       // Port 8: CW
    backTopMotor.move_velocity(INTAKE_SPEED);          // Port 9: CW
    frontTopMotor.move(0);                             // Port 10: Off
}

/**
 * L2: Intake to top storage
 */
void intakeTopStorage() {
    frontMiddleMotor.move_velocity(INTAKE_SPEED);      // Port 6: CW
    backMiddleMotor.move_velocity(-INTAKE_SPEED);      // Port 8: CCW
    frontTopMotor.move_velocity(INTAKE_SPEED);         // Port 10: CW
    backTopMotor.move_velocity(INTAKE_SPEED);          // Port 9: CW
}

/**
 * D-Pad Up: Port 6 CCW
 */
void motor6CCW() {
    frontMiddleMotor.move_velocity(-INTAKE_SPEED);     // Port 6: CCW
}

/**
 * D-Pad Down: Port 9 CCW
 */
void motor9CCW() {
    backTopMotor.move_velocity(-INTAKE_SPEED);         // Port 9: CCW
}

/**
 * D-Pad Left: Port 8 CW
 */
void motor8CW() {
    backMiddleMotor.move_velocity(INTAKE_SPEED);       // Port 8: CW
}

/**
 * D-Pad Right: Port 10 CW
 */
void motor10CW() {
    frontTopMotor.move_velocity(INTAKE_SPEED);         // Port 10: CW
}

// ==================== PNEUMATIC CONTROL ====================

/**
 * Toggle scooper pneumatic (Port D)
 */
void toggleScooper() {
    scooperDeployed = !scooperDeployed;
    scooper.set_value(scooperDeployed);
}

/**
 * Deploy scooper
 */
void deployScooper() {
    scooperDeployed = true;
    scooper.set_value(true);
}

/**
 * Retract scooper
 */
void retractScooper() {
    scooperDeployed = false;
    scooper.set_value(false);
}

/**
 * Toggle piston2 pneumatic (Port A)
 */
void togglePiston2() {
    piston2Deployed = !piston2Deployed;
    piston2.set_value(piston2Deployed);
}

/**
 * Extend piston2
 */
void extendPiston2() {
    piston2Deployed = true;
    piston2.set_value(true);
}

/**
 * Retract piston2
 */
void retractPiston2() {
    piston2Deployed = false;
    piston2.set_value(false);
}

// ==================== AUTONOMOUS HELPER FUNCTIONS ====================

/**
 * Turn to absolute heading using LemLib
 * @param degrees Target heading in degrees
 * @param timeout Maximum time to wait (ms)
 */
void turnToHeading(double degrees, int timeout = 2000) {
    chassis.turnToHeading(degrees, timeout);
}

/**
 * Move forward a specific distance using LemLib
 * @param inches Distance to move in inches
 * @param timeout Maximum time to wait (ms)
 */
void moveForward(double inches, int timeout = 3000) {
    chassis.moveToPoint(0, inches, timeout);
}

/**
 * Run intake for specified duration
 * @param duration Time to run intake in milliseconds
 */
void autoIntakeTop(int duration) {
    intakeTop();
    pros::delay(duration);
    stopIntake();
}

// ==================== INITIALIZATION ====================

void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Initializing...");
    
    // Calibrate IMU - THIS IS CRITICAL
    pros::lcd::set_text(2, "Calibrating IMU...");
    imu.reset();
    
    // Wait for IMU to calibrate (takes about 2 seconds)
    int timeout = 0;
    while (imu.is_calibrating() && timeout < 3000) {
        pros::delay(10);
        timeout += 10;
    }
    
    if (imu.is_calibrating()) {
        pros::lcd::set_text(2, "IMU CALIBRATION FAILED!");
    } else {
        pros::lcd::set_text(2, "IMU Calibrated!");
    }
    
    // Initialize chassis
    chassis.calibrate();
    
    // Set initial states
    retractScooper();
    retractPiston2();
    
    // Reset encoders
    verticalEncoder.reset();
    horizontalEncoder.reset();
    
    pros::delay(500);
    pros::lcd::set_text(3, "Ready!");
}

void disabled() {}

void competition_initialize() {}

// ==================== TEST FUNCTION 1: ENCODER TEST ====================

/**
 * Test encoders by displaying their raw values
 * Call this during driver control by pressing a button
 */
void testEncoders() {
    pros::lcd::clear();
    pros::lcd::set_text(0, "=== ENCODER TEST ===");
    
    for (int i = 0; i < 100; i++) {  // Run for 10 seconds
        int verticalValue = verticalEncoder.get_value();
        int horizontalValue = horizontalEncoder.get_value();
        double imuHeading = imu.get_rotation();
        
        pros::lcd::set_text(1, ("Vertical: " + std::to_string(verticalValue)).c_str());
        pros::lcd::set_text(2, ("Horizontal: " + std::to_string(horizontalValue)).c_str());
        pros::lcd::set_text(3, ("IMU Heading: " + std::to_string(imuHeading).substr(0, 6)).c_str());
        pros::lcd::set_text(4, "Push robot to test!");
        pros::lcd::set_text(5, "Press A to exit");
        
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            break;
        }
        
        pros::delay(100);
    }
}

// ==================== TEST FUNCTION 2: DRIVE STRAIGHT TEST ====================

/**
 * Drive straight forward 24 inches
 * Tests basic autonomous movement
 */
void testDriveStraight() {
    pros::lcd::clear();
    pros::lcd::set_text(0, "=== DRIVE STRAIGHT TEST ===");
    pros::lcd::set_text(1, "Moving forward 24in...");
    
    // Set starting position
    chassis.setPose(0, 0, 0);
    
    // Drive forward 24 inches
    chassis.moveToPoint(0, 24, 3000);
    
    // Check final position
    lemlib::Pose finalPose = chassis.getPose();
    pros::lcd::set_text(2, ("Final Y: " + std::to_string(finalPose.y).substr(0, 6)).c_str());
    pros::lcd::set_text(3, "Should be near 24");
    
    pros::delay(2000);
}

// ==================== TEST FUNCTION 3: TURN TEST ====================

/**
 * Turn 90 degrees to test heading accuracy
 */
void testTurn90() {
    pros::lcd::clear();
    pros::lcd::set_text(0, "=== TURN 90 TEST ===");
    pros::lcd::set_text(1, "Turning 90 degrees...");
    
    // Set starting position
    chassis.setPose(0, 0, 0);
    
    // Turn 90 degrees
    chassis.turnToHeading(90, 2000);
    
    // Check final heading
    lemlib::Pose finalPose = chassis.getPose();
    pros::lcd::set_text(2, ("Final Heading: " + std::to_string(finalPose.theta).substr(0, 6)).c_str());
    pros::lcd::set_text(3, "Should be near 90");
    
    pros::delay(2000);
}

// ==================== TEST FUNCTION 4: SQUARE PATTERN TEST ====================

/**
 * Drive in a 24" square to test overall accuracy
 * Robot should return to starting position
 */
void testSquarePattern() {
    pros::lcd::clear();
    pros::lcd::set_text(0, "=== SQUARE PATTERN TEST ===");
    
    // Set starting position
    chassis.setPose(0, 0, 0);
    setDriveBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    
    // Side 1: Move forward 24"
    pros::lcd::set_text(1, "Side 1: Forward 24in");
    chassis.moveToPoint(0, 24, 3000);
    pros::delay(500);
    
    // Turn 90° left
    pros::lcd::set_text(1, "Turn 1: 90 degrees");
    chassis.turnToHeading(90, 2000);
    pros::delay(500);
    
    // Side 2: Move forward 24"
    pros::lcd::set_text(1, "Side 2: Forward 24in");
    lemlib::Pose pose2 = chassis.getPose();
    chassis.moveToPoint(pose2.x + 24, pose2.y, 3000);
    pros::delay(500);
    
    // Turn 90° left (now facing 180°)
    pros::lcd::set_text(1, "Turn 2: 180 degrees");
    chassis.turnToHeading(180, 2000);
    pros::delay(500);
    
    // Side 3: Move forward 24"
    pros::lcd::set_text(1, "Side 3: Forward 24in");
    lemlib::Pose pose3 = chassis.getPose();
    chassis.moveToPoint(pose3.x, pose3.y - 24, 3000);
    pros::delay(500);
    
    // Turn 90° left (now facing 270° or -90°)
    pros::lcd::set_text(1, "Turn 3: 270 degrees");
    chassis.turnToHeading(270, 2000);
    pros::delay(500);
    
    // Side 4: Move forward 24" (back to start)
    pros::lcd::set_text(1, "Side 4: Forward 24in");
    lemlib::Pose pose4 = chassis.getPose();
    chassis.moveToPoint(pose4.x - 24, pose4.y, 3000);
    pros::delay(500);
    
    // Turn back to 0°
    pros::lcd::set_text(1, "Turn 4: Back to 0");
    chassis.turnToHeading(0, 2000);
    
    // Display final position
    lemlib::Pose finalPose = chassis.getPose();
    pros::lcd::set_text(2, ("Final X: " + std::to_string(finalPose.x).substr(0, 6)).c_str());
    pros::lcd::set_text(3, ("Final Y: " + std::to_string(finalPose.y).substr(0, 6)).c_str());
    pros::lcd::set_text(4, ("Final Heading: " + std::to_string(finalPose.theta).substr(0, 6)).c_str());
    pros::lcd::set_text(5, "Should be near (0, 0, 0)");
    
    setDriveBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    pros::delay(5000);
}

// ==================== TEST FUNCTION 5: CONTINUOUS POSITION DISPLAY ====================

/**
 * Continuously display position while driving
 * Useful for checking if odometry is tracking correctly
 */
void testContinuousDisplay() {
    pros::lcd::clear();
    pros::lcd::set_text(0, "=== POSITION TRACKING ===");
    pros::lcd::set_text(5, "Press A to exit");
    
    chassis.setPose(0, 0, 0);
    
    while (!master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
        lemlib::Pose pose = chassis.getPose();
        
        pros::lcd::set_text(1, ("X: " + std::to_string(pose.x).substr(0, 8) + " in").c_str());
        pros::lcd::set_text(2, ("Y: " + std::to_string(pose.y).substr(0, 8) + " in").c_str());
        pros::lcd::set_text(3, ("Heading: " + std::to_string(pose.theta).substr(0, 8) + " deg").c_str());
        pros::lcd::set_text(4, "Drive robot around");
        
        pros::delay(50);
    }
}

// ==================== AUTONOMOUS ====================
// ==================== AUTONOMOUS HELPER FUNCTIONS ====================

/**
 * Autonomous routine for LEFT side starting position
 * Robot starts in left Park Zone, aligned with 3 blocks near center goals
 * 
 * Field Measurements (from game manual):
 * - Park Zone depth: ~17 inches
 * - Distance to center blocks: ~20-24 inches from start
 * - Center Goal distance: ~30-36 inches from start  
 * - Loader position: Back at ~12-15 inches from start
 * - Long Goal: ~48 inches from start after 180° turn
 */
void autonomousLeft() {
    pros::lcd::set_text(1, "=== AUTO: LEFT SIDE ===");
    
    // STEP 1: Deploy scooper and drive to 3 blocks near center
    pros::lcd::set_text(2, "1: Deploy & Approach");
    deployScooper();
    pros::delay(200);
    
    // Drive forward ~22 inches to reach the 3 blocks
    // Park Zone is 16.86" deep, blocks are ~5" beyond
    chassis.moveToPoint(0, 22, 3000, {.forwards = true});
    chassis.waitUntilDone();
    pros::delay(100);
    
    // STEP 2: Intake the 3 blocks
    pros::lcd::set_text(2, "2: Intake 3 blocks");
    intakeMiddle();  // R1 function
    pros::delay(1800);  // Give time to intake all 3 blocks
    stopIntake();
    pros::delay(100);
    
    // STEP 3: Slight rotation RIGHT to align with upper center goal
    pros::lcd::set_text(2, "3: Align to goal");
    chassis.turnToHeading(12, 2000);  // Turn 12° right for alignment
    chassis.waitUntilDone();
    pros::delay(100);
    
    // STEP 4: Drive up to center goal and outtake (top center goal)
    pros::lcd::set_text(2, "4: Score center goal");
    lemlib::Pose currentPose = chassis.getPose();
    chassis.moveToPoint(currentPose.x, currentPose.y + 10, 2500, {.forwards = true});  // Drive 10" closer
    chassis.waitUntilDone();
    
    intakeMiddle();  // Middle outtake for top center goal
    pros::delay(1600);  // Outtake for 1.6 seconds
    stopIntake();
    pros::delay(100);
    
    // STEP 5: Drive straight back to align with loader
    pros::lcd::set_text(2, "5: Back to loader");
    chassis.turnToHeading(0, 2000);  // Face forward again (0°)
    chassis.waitUntilDone();
    
    chassis.moveToPoint(0, 14, 3000, {.forwards = false});  // Back up to loader position
    chassis.waitUntilDone();
    pros::delay(100);
    
    // STEP 6: Turn LEFT slightly to align with loader column
    pros::lcd::set_text(2, "6: Align loader");
    chassis.turnToHeading(-8, 2000);  // Turn 8° left for loader alignment
    chassis.waitUntilDone();
    pros::delay(100);
    
    // STEP 7: Drive into loader to get 3 blue blocks
    pros::lcd::set_text(2, "7: Load 3 blocks");
    currentPose = chassis.getPose();
    chassis.moveToPoint(currentPose.x - 4, currentPose.y, 2000, {.forwards = true});  // Drive into loader
    chassis.waitUntilDone();
    
    intakeMiddle();  // Start intake
    pros::delay(2200);  // Wait for 3 blocks to drop and intake (estimated time)
    stopIntake();
    pros::delay(100);
    
    // STEP 8: Back out of loader
    pros::lcd::set_text(2, "8: Exit loader");
    currentPose = chassis.getPose();
    chassis.moveToPoint(currentPose.x + 5, currentPose.y, 2000, {.forwards = false});
    chassis.waitUntilDone();
    retractScooper();  // Retract scooper
    pros::delay(100);
    
    // STEP 9: Turn 180° to face long goal
    pros::lcd::set_text(2, "9: Turn to long goal");
    chassis.turnToHeading(180, 2500);
    chassis.waitUntilDone();
    pros::delay(100);
    
    // STEP 10: Drive to long goal and outtake
    pros::lcd::set_text(2, "10: Score long goal");
    currentPose = chassis.getPose();
    chassis.moveToPoint(currentPose.x, currentPose.y - 12, 3000, {.forwards = true});  // Drive to long goal
    chassis.waitUntilDone();
    
    intakeTop();  // R2 function - outtake top
    pros::delay(1800);
    stopIntake();
    
    pros::lcd::set_text(2, "=== COMPLETE ===");
}

/**
 * Autonomous routine for RIGHT side starting position
 * Robot starts in right Park Zone, aligned with 3 blocks near center goals
 * Same measurements as left side, but mirrored
 */
void autonomousRight() {
    pros::lcd::set_text(1, "=== AUTO: RIGHT SIDE ===");
    
    // STEP 1: Deploy scooper and drive to 3 blocks near center
    pros::lcd::set_text(2, "1: Deploy & Approach");
    deployScooper();
    pros::delay(200);
    
    // Drive forward ~22 inches to reach the 3 blocks
    chassis.moveToPoint(0, 22, 3000, {.forwards = true});
    chassis.waitUntilDone();
    pros::delay(100);
    
    // STEP 2: Intake the 3 blocks
    pros::lcd::set_text(2, "2: Intake 3 blocks");
    intakeMiddle();  // R1 function
    pros::delay(1800);  // Give time to intake all 3 blocks
    stopIntake();
    pros::delay(100);
    
    // STEP 3: Slight rotation LEFT to align with lower center goal
    pros::lcd::set_text(2, "3: Align to goal");
    chassis.turnToHeading(-12, 2000);  // Turn 12° left (opposite of left side)
    chassis.waitUntilDone();
    pros::delay(100);
    
    // STEP 4: Drive up to center goal and outtake (bottom center goal)
    pros::lcd::set_text(2, "4: Score center goal");
    lemlib::Pose currentPose = chassis.getPose();
    chassis.moveToPoint(currentPose.x, currentPose.y + 10, 2500, {.forwards = true});  // Drive 10" closer
    chassis.waitUntilDone();
    
    intakeBottomStorage();  // L1 function - bottom outtake for lower center goal
    pros::delay(1600);  // Outtake for 1.6 seconds
    stopIntake();
    pros::delay(100);
    
    // STEP 5: Drive straight back to align with loader
    pros::lcd::set_text(2, "5: Back to loader");
    chassis.turnToHeading(0, 2000);  // Face forward again
    chassis.waitUntilDone();
    
    chassis.moveToPoint(0, 14, 3000, {.forwards = false});  // Back up to loader position
    chassis.waitUntilDone();
    pros::delay(100);
    
    // STEP 6: Turn RIGHT slightly to align with loader column
    pros::lcd::set_text(2, "6: Align loader");
    chassis.turnToHeading(8, 2000);  // Turn 8° right (opposite of left side)
    chassis.waitUntilDone();
    pros::delay(100);
    
    // STEP 7: Drive into loader to get 3 blue blocks
    pros::lcd::set_text(2, "7: Load 3 blocks");
    currentPose = chassis.getPose();
    chassis.moveToPoint(currentPose.x + 4, currentPose.y, 2000, {.forwards = true});  // Drive into loader (positive X)
    chassis.waitUntilDone();
    
    intakeMiddle();  // Start intake
    pros::delay(2200);  // Wait for 3 blocks to drop and intake
    stopIntake();
    pros::delay(100);
    
    // STEP 8: Back out of loader
    pros::lcd::set_text(2, "8: Exit loader");
    currentPose = chassis.getPose();
    chassis.moveToPoint(currentPose.x - 5, currentPose.y, 2000, {.forwards = false});
    chassis.waitUntilDone();
    retractScooper();  // Retract scooper
    pros::delay(100);
    
    // STEP 9: Turn 180° to face long goal
    pros::lcd::set_text(2, "9: Turn to long goal");
    chassis.turnToHeading(180, 2500);
    chassis.waitUntilDone();
    pros::delay(100);
    
    // STEP 10: Drive to long goal and outtake
    pros::lcd::set_text(2, "10: Score long goal");
    currentPose = chassis.getPose();
    chassis.moveToPoint(currentPose.x, currentPose.y - 12, 3000, {.forwards = true});  // Drive to long goal
    chassis.waitUntilDone();
    
    intakeTop();  // R2 function - outtake top
    pros::delay(1800);
    stopIntake();
    
    pros::lcd::set_text(2, "=== COMPLETE ===");
}

// ==================== MAIN AUTONOMOUS FUNCTION ====================

void autonomous() {
    autonomousMode = true;
    setDriveBrakeMode(pros::E_MOTOR_BRAKE_HOLD);  // Use HOLD for precise autonomous
    
    pros::lcd::clear();
    
    // Set starting position (0, 0, 0)
    // Robot is in Park Zone, facing forward
    chassis.setPose(0, 0, 0);
    
    // ===== SELECT YOUR STARTING SIDE =====
    // Uncomment ONE of the following lines:
    // NOTE: The autonomous routines are based on this: https://www.instagram.com/reel/DUJWBtLkWIO/?igsh=MWtqd2JnbjU4MWhvdg%3D%3D
    autonomousLeft();   // Use this for LEFT side start
    // autonomousRight();  // Use this for RIGHT side start
    
    // =====================================
    
    autonomousMode = false;
    setDriveBrakeMode(pros::E_MOTOR_BRAKE_COAST);  // Back to coast for driver
}


// ==================== DRIVER CONTROL ====================

void opcontrol() {
    autonomousMode = false;
    setDriveBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    
    // Display task for debugging
    pros::Task displayTask([&]() {
        while (true) {
            lemlib::Pose pose = chassis.getPose();
            
            pros::lcd::set_text(0, "=== DRIVER CONTROL ===");
            pros::lcd::set_text(1, ("X: " + std::to_string(pose.x).substr(0, 6)).c_str());
            pros::lcd::set_text(2, ("Y: " + std::to_string(pose.y).substr(0, 6)).c_str());
            pros::lcd::set_text(3, ("Heading: " + std::to_string(pose.theta).substr(0, 6)).c_str());
            pros::lcd::set_text(4, scooperDeployed ? "Scooper: DEPLOYED" : "Scooper: RETRACTED");
            pros::lcd::set_text(5, piston2Deployed ? "Piston2: EXTENDED" : "Piston2: RETRACTED");
            
            pros::delay(50);
        }
    });
    
    // Main control loop
    while (true) {
        // ===== DRIVETRAIN CONTROL =====
        // Axis3 = Left stick Y (forward/backward)
        // Axis1 = Left stick X (turning)
        int forward = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        
        omniDrive(forward, 0, turn);
        
        // ===== MOTOR BUTTON CONTROLS =====
        
        // Check which button is pressed and execute corresponding function
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intakeMiddle();  // R1: Intake middle
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intakeTop();  // R2: Intake top (shoot to top)
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intakeBottomStorage();  // L1: Intake to bottom storage
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intakeTopStorage();  // L2: Intake to top storage
        }
        // No buttons: Stop all motors
        else {
            stopIntake();
        }
        
        // ===== PNEUMATIC CONTROLS =====
        
        // Port D (Scooper): X to extend, B to retract
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            deployScooper();
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            retractScooper();
        }
        
        // Port A (Piston2): A to extend, Y to retract
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            extendPiston2();
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
            retractPiston2();
        }

        /*
        // ===== TEST FUNCTIONS (Remove after testing) =====
        // Press UP + A together to test encoders
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP) && 
            master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            testEncoders();
        }
        
        // Press UP + B together to test drive straight
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP) && 
            master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            testDriveStraight();
        }
        
        // Press UP + X together to test turn
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP) && 
            master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            testTurn90();
        }
        
        // Press UP + Y together to test square pattern
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP) && 
            master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            testSquarePattern();
        }
        
        // Press DOWN + A together for continuous position display
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN) && 
            master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            testContinuousDisplay();
        }
         */
        
        // Main loop delay
        pros::delay(10);
    }
}