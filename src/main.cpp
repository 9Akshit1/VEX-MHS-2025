/**
 * main.cpp - VEX Robot Control System
 * 
 * Robot Configuration:
 * - Tank drive with omni wheels (3 motors per side)
 * - Complex intake/elevator system (3 motors)
 * - Pneumatic scooper
 * - Odometry system (vertical + horizontal encoders)
 * 
 * Control Scheme:
 * - Left Stick Y: Forward/Backward
 * - Right Stick X: Strafe Left/Right
 * - Left Sti☺ck X: Turning
 * - R1: Intake (bring game piece up)
 * - R2: Outtake through top rollers
 * - L1: Outtake through middle roller
 * - L2: Outtake through back rollers (to storage)
 * - Up: Carry down and outtake through bottom
 * - Down: Reverse all intake motors
 * - X: Toggle scooper
 * - Y: Reset odometry
 * - A: Recalibrate encoders
 */

#include "main.h"
#include "lemlib/api.hpp"
#include <cmath>

// ==================== MOTOR DECLARATIONS ====================

// DRIVETRAIN - 6 motors (3 per side, linear tank configuration with omni wheels)
pros::Motor leftFront(1, pros::MotorGearset::blue);      // Port 1
pros::Motor leftMiddle(2, pros::MotorGearset::blue);     // Port 2
pros::Motor leftBack(3, pros::MotorGearset::blue);       // Port 3

pros::Motor rightFront(-4, pros::MotorGearset::blue);    // Port 4 (reversed)
pros::Motor rightMiddle(-5, pros::MotorGearset::blue);   // Port 5 (reversed)
pros::Motor rightBack(-6, pros::MotorGearset::blue);     // Port 6 (reversed)

// Motor groups for LemLib
pros::MotorGroup leftMotors({1, 2, 3}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({-4, -5, -6}, pros::MotorGearset::blue);

// INTAKE/ELEVATOR SYSTEM - 3 motors
// Motor 7: Front top motor (controls front top 2 rollers)  --- port 7 wasnt working, so port 10 is used instead
pros::Motor frontTopMotor(10, pros::MotorGearset::green);

// Motor 8: Back top motor (controls back top roller + front middle roller via chain)
pros::Motor backTopMotor(8, pros::MotorGearset::green);

// Motor 9: Back middle motor (controls back middle roller)
pros::Motor backMiddleMotor(9, pros::MotorGearset::green);

// Note: The bottom front roller (actual intake) is chained to backTopMotor

// ==================== PNEUMATICS ====================
pros::adi::DigitalOut scooper('A');  // Pneumatic solenoid for scooper

// ==================== SENSORS ====================
pros::adi::Encoder verticalEncoder('B', 'C', false);    // Vertical tracking wheel
pros::adi::Encoder horizontalEncoder('D', 'E', false);  // Horizontal tracking wheel

// ==================== CONTROLLER ====================
pros::Controller master(pros::E_CONTROLLER_MASTER);

// ==================== ROBOT CONFIGURATION ====================

// Tracking wheel specifications
const double TRACKING_WHEEL_DIAMETER = 2.75;  // inches
const double TRACKING_WHEEL_DISTANCE = 360.0 / (TRACKING_WHEEL_DIAMETER * M_PI);  // ticks per inch

// Robot physical dimensions
const double WHEEL_DIAMETER = 3.25;           // inches (omni wheels)
const double TRACK_WIDTH = 12.0;              // inches (distance between left and right wheels)
const double WHEELBASE_LENGTH = 12.0;         // inches (front to back distance)

// Tracking wheel offsets from center of rotation
const double VERTICAL_OFFSET = 2.0;           // inches forward from center
const double HORIZONTAL_OFFSET = 5.5;         // inches to the side from center

// Motor speeds (RPM)
const int INTAKE_SPEED = 200;
const int OUTTAKE_SPEED = 150;

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
lemlib::ControllerSettings lateralController(
    10,   // kP
    0,    // kI
    55,   // kD
    3,    // anti-windup
    1,    // small error range
    100,  // small error timeout
    3,    // large error range
    500,  // large error timeout
    20    // slew rate
);

lemlib::ControllerSettings angularController(
    4,    // kP
    0,    // kI
    40,   // kD
    3,    // anti-windup
    1,    // small error range
    100,  // small error timeout
    3,    // large error range
    500,  // large error timeout
    0     // slew rate
);

// Drivetrain configuration
lemlib::Drivetrain drivetrain(
    &leftMotors,
    &rightMotors,
    TRACK_WIDTH,
    lemlib::Omniwheel::NEW_325,  // 3.25" omni wheels
    360,  // RPM (blue cartridge = 600 RPM, but using conservative value)
    2     // horizontal drift correction
);

// Odometry sensors (using encoders only, no IMU)
lemlib::OdomSensors sensors(
    &vertical,     // vertical tracking wheel
    nullptr,       // second vertical wheel (not used)
    &horizontal,   // horizontal tracking wheel
    nullptr,       // second horizontal wheel (not used)
    nullptr        // no IMU
);

// Create the chassis
lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);

// ==================== GLOBAL STATE VARIABLES ====================
bool scooperDeployed = false;
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
 * Tank drive with omni wheel strafing capability
 * Combines tank drive with lateral movement
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
    
    // For tank drive with omni wheels:
    // - Forward/backward: both sides move same direction
    // - Strafing: all wheels move same direction (omni wheels allow this)
    // - Turning: sides move opposite directions
    
    int leftPower = forward + strafe + turn;
    int rightPower = forward - strafe - turn;
    
    // Normalize if any value exceeds motor limits
    int maxPower = std::max(abs(leftPower), abs(rightPower));
    if (maxPower > 127) {
        double scale = 127.0 / maxPower;
        leftPower *= scale;
        rightPower *= scale;
    }
    
    // Apply to all motors
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
 * Stop all intake/elevator motors
 */
void stopIntake() {
    frontTopMotor.move_velocity(0);
    backTopMotor.move_velocity(0);
    backMiddleMotor.move_velocity(0);
}

/**
 * INTAKE MODE: Bring game piece up through the system
 * - Bottom intake roller pulls in
 * - All rollers move to carry piece upward
 */
void intakeUp() {
    frontTopMotor.move_velocity(INTAKE_SPEED);      // Front top rollers forward (reverse)
    backTopMotor.move_velocity(INTAKE_SPEED);       // Back top + front middle + bottom intake (reverse)
    backMiddleMotor.move_velocity(-INTAKE_SPEED);    // Back middle roller forward
}

/**
 * OUTTAKE TOP: Eject through front top 2 rollers
 * Used for scoring at high positions
 */
void outtakeTop() {
    frontTopMotor.move_velocity(OUTTAKE_SPEED);    // Front top rollers reverse (outtake)
    backTopMotor.move_velocity(-INTAKE_SPEED);       // Keep feeding up
    backMiddleMotor.move_velocity(-INTAKE_SPEED);    // Keep feeding up
}

/**
 * OUTTAKE MIDDLE: Eject through front middle roller
 * Controlled by backTopMotor (chained connection)
 */
void outtakeMiddle() {
    frontTopMotor.move_velocity(0);                 // Keep top rollers stopped
    backTopMotor.move_velocity(OUTTAKE_SPEED);     // Reverse to push out middle
    backMiddleMotor.move_velocity(-INTAKE_SPEED);    // Feed from below
}

/**
 * OUTTAKE BACK: Eject through back rollers into storage box
 * Both back motors reverse to push piece backward
 */
void outtakeBack() {
    frontTopMotor.move_velocity(0);                    // Front stays neutral
    backTopMotor.move_velocity(OUTTAKE_SPEED);        // Back top reverses
    backMiddleMotor.move_velocity(-OUTTAKE_SPEED);     // Back middle 
}

/**
 * CARRY DOWN AND OUTTAKE BOTTOM: Lower piece and eject through bottom intake
 * All motors reverse to bring piece down and out
 */
void outtakeBottom() {
    frontTopMotor.move_velocity(0);    // Front stays neutral
    backTopMotor.move_velocity(OUTTAKE_SPEED);     // Reverse bottom intake
    backMiddleMotor.move_velocity(OUTTAKE_SPEED);  // Bring down (reverse)
}

/**
 * REVERSE ALL: Emergency reverse of entire intake system
 */
void reverseIntake() {
    frontTopMotor.move_velocity(0);    // opposite of intakeUp()
    backTopMotor.move_velocity(-INTAKE_SPEED);    // opposite of intakeUp()
    backMiddleMotor.move_velocity(INTAKE_SPEED);    // opposite of intakeUp()
}

// ==================== SCOOPER CONTROL ====================

/**
 * Deploy scooper (extend pneumatic)
 */
void deployScooper() {
    scooper.set_value(true);
    scooperDeployed = true;
}

/**
 * Retract scooper (retract pneumatic)
 */
void retractScooper() {
    scooper.set_value(false);
    scooperDeployed = false;
}

/**
 * Toggle scooper state
 */
void toggleScooper() {
    if (scooperDeployed) {
        retractScooper();
    } else {
        deployScooper();
    }
}

// ==================== AUTONOMOUS MOVEMENT FUNCTIONS ====================

/**
 * Move forward by distance (inches)
 * Positive = forward, Negative = backward
 */
void moveForward(double distance, int timeout = 3000) {
    lemlib::Pose current = chassis.getPose();
    double targetX = current.x + distance * cos(current.theta * M_PI / 180.0);
    double targetY = current.y + distance * sin(current.theta * M_PI / 180.0);
    
    chassis.moveToPose(targetX, targetY, current.theta, timeout, {.minSpeed = 40});
}

/**
 * Move backward by distance (inches)
 */
void moveBackward(double distance, int timeout = 3000) {
    moveForward(-distance, timeout);
}

/**
 * Strafe right by distance (inches)
 */
void strafeRight(double distance, int timeout = 3000) {
    lemlib::Pose current = chassis.getPose();
    double angle = (current.theta - 90) * M_PI / 180.0;  // Perpendicular to heading
    double targetX = current.x + distance * cos(angle);
    double targetY = current.y + distance * sin(angle);
    
    chassis.moveToPose(targetX, targetY, current.theta, timeout, {.minSpeed = 40});
}

/**
 * Strafe left by distance (inches)
 */
void strafeLeft(double distance, int timeout = 3000) {
    strafeRight(-distance, timeout);
}

/**
 * Turn to absolute heading (degrees)
 * 0° = forward, 90° = right, 180° = back, 270° = left
 */
void turnToHeading(double heading, int timeout = 2000) {
    chassis.turnToHeading(heading, timeout, {.minSpeed = 30});
}

/**
 * Turn relative to current heading (degrees)
 * Positive = clockwise, Negative = counter-clockwise
 */
void turnRelative(double degrees, int timeout = 2000) {
    lemlib::Pose current = chassis.getPose();
    double targetHeading = current.theta + degrees;
    chassis.turnToHeading(targetHeading, timeout, {.minSpeed = 30});
}

/**
 * Turn clockwise by degrees
 */
void turnRight(double degrees, int timeout = 2000) {
    turnRelative(degrees, timeout);
}

/**
 * Turn counter-clockwise by degrees
 */
void turnLeft(double degrees, int timeout = 2000) {
    turnRelative(-degrees, timeout);
}

/**
 * Move to specific coordinates (x, y, heading)
 */
void moveToPose(double x, double y, double heading, int timeout = 3000) {
    chassis.moveToPose(x, y, heading, timeout, {.minSpeed = 40});
}

/**
 * Move to specific point while maintaining current heading
 */
void moveToPoint(double x, double y, int timeout = 3000) {
    lemlib::Pose current = chassis.getPose();
    chassis.moveToPose(x, y, current.theta, timeout, {.minSpeed = 40});
}

/**
 * Turn to face a specific point (x, y)
 */
void turnToPoint(double x, double y, int timeout = 2000) {
    chassis.turnToPoint(x, y, timeout, {.minSpeed = 30});
}

/**
 * Move along a path of points
 */
void followPath(std::vector<lemlib::Pose> path, int timeout = 5000) {
    for (const auto& point : path) {
        chassis.moveToPose(point.x, point.y, point.theta, timeout, {.minSpeed = 40});
    }
}

// ==================== AUTONOMOUS ACTION FUNCTIONS ====================

/**
 * Intake for specified duration (milliseconds)
 */
void autoIntake(int duration) {
    intakeUp();
    pros::delay(duration);
    stopIntake();
}

/**
 * Outtake through top for specified duration
 */
void autoOuttakeTop(int duration) {
    outtakeTop();
    pros::delay(duration);
    stopIntake();
}

/**
 * Outtake through middle for specified duration
 */
void autoOuttakeMiddle(int duration) {
    outtakeMiddle();
    pros::delay(duration);
    stopIntake();
}

/**
 * Outtake through back for specified duration
 */
void autoOuttakeBack(int duration) {
    outtakeBack();
    pros::delay(duration);
    stopIntake();
}

/**
 * Outtake through bottom for specified duration
 */
void autoOuttakeBottom(int duration) {
    outtakeBottom();
    pros::delay(duration);
    stopIntake();
}

/**
 * Deploy scooper, wait, then retract
 */
void autoScooperSequence(int deployDuration = 500) {
    deployScooper();
    pros::delay(deployDuration);
    retractScooper();
}

/**
 * Intake sequence: Deploy scooper, intake, retract scooper
 */
void autoFullIntake(int intakeDuration = 1500) {
    deployScooper();
    intakeUp();
    pros::delay(intakeDuration);
    stopIntake();
    retractScooper();
}

// ==================== INITIALIZATION ====================

void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "=== INITIALIZING ===");
    
    // Reset encoders
    pros::lcd::set_text(2, "Resetting encoders...");
    verticalEncoder.reset();
    horizontalEncoder.reset();
    pros::delay(200);
    
    // Initialize chassis position
    chassis.calibrate();
    chassis.setPose(0, 0, 0);
    
    // Set brake modes
    setDriveBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    
    // Initialize pneumatics
    retractScooper();
    
    pros::lcd::set_text(3, "Ready!");
    pros::delay(500);
}

void disabled() {
    stopDrive();
    stopIntake();
    retractScooper();
}

void competition_initialize() {
    // Can add autonomous selector here if needed
}

// ==================== AUTONOMOUS ====================

void autonomous() {
    autonomousMode = true;
    pros::lcd::clear();
    pros::lcd::set_text(1, "=== AUTONOMOUS ===");
    
    // Verify odometry is working
    lemlib::Pose startPose = chassis.getPose();
    if (std::isnan(startPose.x) || std::isnan(startPose.y)) {
        pros::lcd::set_text(2, "ERROR: Odometry NaN");
        return;
    }
    
    // ===== EXAMPLE AUTONOMOUS ROUTINE =====
    // Replace this with your actual autonomous routine
    
    // Example: Move forward, deploy scooper, intake, move back, score
    
    moveForward(24);           // Move forward 24 inches
    pros::delay(200);
    
    deployScooper();           // Deploy scooper
    pros::delay(300);
    
    autoIntake(1500);          // Intake for 1.5 seconds
    
    retractScooper();          // Retract scooper
    pros::delay(200);
    
    moveBackward(12);          // Move back 12 inches
    pros::delay(200);
    
    turnRight(90);             // Turn 90 degrees clockwise
    pros::delay(200);
    
    moveForward(18);           // Move forward to scoring position
    pros::delay(200);
    
    autoOuttakeTop(1000);      // Score through top rollers
    
    pros::lcd::set_text(2, "=== COMPLETE ===");
    autonomousMode = false;
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
            
            pros::delay(50);
        }
    });
    
    // Main control loop
    while (true) {
        // ===== DRIVETRAIN CONTROL =====
        int forward = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int strafe = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        
        omniDrive(forward, strafe, turn);
        
        // ===== INTAKE/ELEVATOR CONTROL =====
        
        // R1: Intake (bring game piece up)
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intakeUp();
        }
        // R2: Outtake through top rollers
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            outtakeTop();
        }
        // L1: Outtake through middle roller
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            outtakeMiddle();
        }
        // L2: Outtake through back rollers (to storage)
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            outtakeBack();
        }
        // Up: Carry down and outtake through bottom
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            outtakeBottom();
        }
        // Down: Reverse all intake motors
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            reverseIntake();
        }
        // No buttons: Stop intake
        else {
            stopIntake();
        }
        
        // ===== SCOOPER CONTROL =====
        
        // X: Toggle scooper
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            toggleScooper();
            master.rumble(".");
        }
        
        // ===== UTILITY CONTROLS =====
        
        // Y: Reset odometry to (0, 0, 0)
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            chassis.setPose(0, 0, 0);
            master.rumble("-");
            master.print(0, 0, "Pose Reset");
        }
        
        // A: Recalibrate encoders
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            master.rumble("--");
            master.print(0, 0, "Recalibrating...");
            
            verticalEncoder.reset();
            horizontalEncoder.reset();
            pros::delay(200);
            
            chassis.setPose(0, 0, 0);
            
            master.rumble("..");
            master.print(0, 0, "Calibrated!");
        }
        
        // Main loop delay
        pros::delay(10);
    }
}