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
 *   - backMiddleMotor: Port 8
 *   - backTopMotor: Port 9 (initial CCW movement at startup)
 * - Pneumatics:
 *   - Scooper: Port D (controlled by X/B)
 * - Sensors:
 *   - None (using physics-based autonomous for movement)
 * 
 * Control Scheme:
 * - Left Stick Y (Axis3): Forward/Backward
 * - Left Stick X (Axis1): Turning (LEFT STICK)
 * - Right Stick X (Axis4): Turning (RIGHT STICK)
 * - R1: Intake middle (Port 6 forward, others run to support)
 * - R2: Intake top (shoot to top - all motors forward)
 * - L1: Intake to bottom storage
 * - L2: Intake to top storage
 * - X: Deploy scooper (Port D)
 * - B: Retract scooper (Port D)
 * - A: Port 6 CCW
 * - Y: Port 6 CW
 * - D-Pad Left: Port 8 CCW
 * - D-Pad Right: Port 10 CCW
 * - D-Pad Up: Port 10 CCW
 * - D-Pad Down: Port 10 CW
 */

#include "main.h"
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

pros::adi::DigitalOut scooper('H');  // Port E - Scooper control (X/B buttons)

// ==================== SENSORS ====================
// (Encoders and IMU removed - using physics-based autonomous)

// ==================== CONTROLLER ====================
pros::Controller master(pros::E_CONTROLLER_MASTER);

// ==================== ROBOT CONFIGURATION ====================

// Robot physical dimensions
const double WHEEL_DIAMETER = 3.25;           // inches (omni wheels)
const double TRACK_WIDTH = 12.5;              // inches (distance between left and right wheels)

// Motor speeds (RPM) - All intake motors are GREEN (200 RPM max)
const int INTAKE_SPEED = 180;      // RPM for intake operations
const int OUTTAKE_SPEED = 140;     // RPM for outtake operations

// Startup movement for backTopMotor
const int STARTUP_BACKTOP_SPEED = 100;        // RPM for initial CCW movement
const int STARTUP_BACKTOP_TIME = 1000;        // ms to run backTopMotor CCW at startup

// Deadzone for joysticks
const int JOYSTICK_DEADZONE = 10;   // Run measureJoystickDeadzone() to tune

// Motor power limits
const int MOTOR_MAX_POWER = 127;

// ===== AUTONOMOUS TUNING PARAMETERS (Physics-Based) =====

// Drivetrain configuration
const int DRIVETRAIN_RPM = 600;                // RPM for BLUE motors
const double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * M_PI;  // inches per revolution

// Speed settings for autonomous (0-127)
const int AUTO_DRIVE_SPEED = 80;               // Motor power for driving
const int AUTO_TURN_SPEED = 60;                // Motor power for turning

// Autonomous routine timings (physics-based: distance = velocity × time)
const int AUTO_DRIVE_TO_BLOCKS = 1500;         // ms to drive to blocks
const int AUTO_INTAKE_TIME = 1000;             // ms to intake 3 blocks
const int AUTO_TURN_45_DEG = 250;              // ms to turn 45 degrees
const int AUTO_APPROACH_GOAL = 500;           // ms to approach goal
const int AUTO_OUTTAKE_TIME = 1500;            // ms to outtake to goal
const int AUTO_BACK_TO_LOADER = 1800;          // ms to back up to loader
const int AUTO_TURN_135_DEG = 1700;            // ms to turn 135 degrees (180-45)
const int AUTO_DRIVE_TO_LOADER = 300;          // ms to drive into loader
const int AUTO_LOADER_TIME = 2200;             // ms for loader to drop blocks
const int AUTO_EXIT_LOADER = 400;              // ms to exit loader
const int AUTO_TURN_180_DEG = 1000;            // ms to turn 180 degrees
const int AUTO_DRIVE_LONG_GOAL = 200;         // ms to drive to long goal
const int AUTO_LONG_GOAL_TIME = 1800;          // ms to outtake to long goal

// Autonomous delay timings
const int AUTO_PNEUMATIC_DELAY = 200;          // ms after pneumatic actuation
const int AUTO_STEP_DELAY = 100;               // ms between autonomous steps

// Autonomous smooth transitions
const int AUTO_SIMULTANEOUS_DELAY = 100;       // ms delay for overlapping actions
const int AUTO_INTAKE_START_DELAY = 200;       // ms delay before starting intake while moving

// Test function parameters
const int TEST_DELAY_BETWEEN_MOVES = 500;      // ms delay between test movements
const int TEST_FINAL_DISPLAY_TIME = 2000;      // ms to display test results

// Display update timing
const int DISPLAY_UPDATE_INTERVAL = 50;        // ms between display updates

// Main loop timing
const int MAIN_LOOP_DELAY = 10;                // ms delay in main control loop

// ==================== MOTOR GROUPS ====================

// Motor groups for easier control
pros::MotorGroup leftMotors({-15, -16, -19}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({1, 2, 3}, pros::MotorGearset::blue);


// ==================== GLOBAL STATE VARIABLES ====================
bool scooperDeployed = false;
bool autonomousMode = false;

// ==================== UTILITY FUNCTIONS ====================

int applyDeadzone(int value, int deadzone = JOYSTICK_DEADZONE) {
    return (abs(value) < deadzone) ? 0 : value;
}

int constrain(int value, int min, int max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

// ==================== DRIVETRAIN CONTROL ====================

void omniDrive(int forward, int strafe, int turn) {
    forward = applyDeadzone(forward);
    strafe = applyDeadzone(strafe);
    turn = applyDeadzone(turn);
    
    int leftPower = forward + strafe + turn;
    int rightPower = forward - strafe - turn;
    
    int maxPower = std::max(abs(leftPower), abs(rightPower));
    if (maxPower > MOTOR_MAX_POWER) {
        double scale = (double)MOTOR_MAX_POWER / maxPower;
        leftPower *= scale;
        rightPower *= scale;
    }
    
    leftFront.move(leftPower);
    leftMiddle.move(leftPower);
    leftBack.move(leftPower);
    
    rightFront.move(rightPower);
    rightMiddle.move(rightPower);
    rightBack.move(rightPower);
}

void stopDrive() {
    leftFront.move(0);
    leftMiddle.move(0);
    leftBack.move(0);
    rightFront.move(0);
    rightMiddle.move(0);
    rightBack.move(0);
}

void setDriveBrakeMode(pros::motor_brake_mode_e mode) {
    leftFront.set_brake_mode(mode);
    leftMiddle.set_brake_mode(mode);
    leftBack.set_brake_mode(mode);
    rightFront.set_brake_mode(mode);
    rightMiddle.set_brake_mode(mode);
    rightBack.set_brake_mode(mode);
}

// ==================== PHYSICS-BASED AUTONOMOUS FUNCTIONS ====================

// Drive forward or backward for a specific time at a given speed
void driveFor(int speed, int time_ms, bool forward = true) {
    int power = forward ? speed : -speed;
    
    leftMotors.move(power);
    rightMotors.move(power);
    
    pros::delay(time_ms);
    
    leftMotors.move(0);
    rightMotors.move(0);
}

// Turn left or right for a specific time at a given speed
void turnFor(int speed, int time_ms, bool right = true) {
    int power = right ? speed : -speed;
    
    leftMotors.move(power);
    rightMotors.move(-power);
    
    pros::delay(time_ms);
    
    leftMotors.move(0);
    rightMotors.move(0);
}


// ==================== INTAKE/ELEVATOR CONTROL ====================

void stopIntake() {
    frontTopMotor.move(0);
    frontMiddleMotor.move(0);
    backTopMotor.move(0);
    backMiddleMotor.move(0);
}

void outakeMiddle() {
    frontMiddleMotor.move_velocity(INTAKE_SPEED);
    backMiddleMotor.move_velocity(-INTAKE_SPEED);
    frontTopMotor.move_velocity(-INTAKE_SPEED);
    backTopMotor.move_velocity(0);
}

void outakeTop() {
    frontMiddleMotor.move_velocity(INTAKE_SPEED);
    backMiddleMotor.move_velocity(-INTAKE_SPEED);
    frontTopMotor.move_velocity(INTAKE_SPEED);
    backTopMotor.move_velocity(-INTAKE_SPEED);
}

void outakeBottom() {
    frontMiddleMotor.move_velocity(-INTAKE_SPEED);
    backMiddleMotor.move_velocity(INTAKE_SPEED);
    frontTopMotor.move_velocity(-INTAKE_SPEED);
    backTopMotor.move_velocity(INTAKE_SPEED);
}

void outakeStorage() {
    frontMiddleMotor.move_velocity(INTAKE_SPEED);
    backMiddleMotor.move_velocity(-INTAKE_SPEED);
    frontTopMotor.move_velocity(0);
    backTopMotor.move_velocity(0);
}

void motor6CCW() {
    frontMiddleMotor.move_velocity(INTAKE_SPEED);
}

void motor6CW() {
    frontMiddleMotor.move_velocity(-INTAKE_SPEED);
}

void motor8CCW() {
    backMiddleMotor.move_velocity(INTAKE_SPEED);
}

void motor8CW() {
    backMiddleMotor.move_velocity(-INTAKE_SPEED);
}

void motor10CCW() {
    frontTopMotor.move_velocity(INTAKE_SPEED);
}

void motor10CW() {
    frontTopMotor.move_velocity(-INTAKE_SPEED);
}

// ==================== PNEUMATIC CONTROL ====================

void toggleScooper() {
    scooperDeployed = !scooperDeployed;
    scooper.set_value(scooperDeployed);
}

void deployScooper() {
    scooperDeployed = false;
    scooper.set_value(false);
}

void retractScooper() {
    scooperDeployed = true;
    scooper.set_value(true);
}

// ==================== AUTONOMOUS HELPER FUNCTIONS ====================

void autoIntakeTop(int duration) {
    outakeTop();
    pros::delay(duration);
    stopIntake();
}

// ==================== INITIALIZATION ====================

void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Initializing...");
    
    // Startup movement for backTopMotor
    pros::lcd::set_text(2, "Moving backTopMotor...");
    backTopMotor.move_velocity(-STARTUP_BACKTOP_SPEED);
    pros::delay(STARTUP_BACKTOP_TIME);
    backTopMotor.move(0);
    
    // Set initial states
    retractScooper();
    
    pros::delay(200);
    pros::lcd::set_text(3, "Ready!");
}


void disabled() {}

void competition_initialize() {}

// ==================== AUTONOMOUS ROUTINES ====================

void autonomousLeft(bool full = true) {
    pros::lcd::set_text(1, "=== AUTO: LEFT ===");
    
    // STEP 1: Retract & approach blocks
    //pros::lcd::set_text(2, "1: Approach");
    retractScooper();
    pros::delay(AUTO_PNEUMATIC_DELAY);
    
    // Drive forward to blocks
    driveFor(AUTO_DRIVE_SPEED, AUTO_DRIVE_TO_BLOCKS * 0.8, true);
    
    // STEP 2: Deploy & intake while moving
    //pros::lcd::set_text(2, "2: Intake");
    deployScooper();
    pros::delay(100);
    
    outakeMiddle();

    driveFor(AUTO_DRIVE_SPEED, AUTO_DRIVE_TO_BLOCKS * 0.2, true);
    
    pros::delay(AUTO_INTAKE_TIME); 
    stopIntake();
    
    // STEP 3: Turn & retract simultaneously
    //pros::lcd::set_text(2, "3: Align");
    turnFor(AUTO_TURN_SPEED, AUTO_TURN_45_DEG, true);  // Turn right 45 degrees
    
    pros::delay(AUTO_SIMULTANEOUS_DELAY);
    retractScooper();
    
    pros::delay(200);
    
    // STEP 4: Score center goal
    //pros::lcd::set_text(2, "4: Score center");
    driveFor(AUTO_DRIVE_SPEED, AUTO_APPROACH_GOAL, true);
    
    outakeBottom();
    pros::delay(AUTO_OUTTAKE_TIME);
    stopIntake();
    
    // STEP 5: Back to loader
    //pros::lcd::set_text(2, "5: To loader");
    driveFor(AUTO_DRIVE_SPEED, AUTO_BACK_TO_LOADER, false);  // Reverse
    
    // STEP 6: Turn & deploy simultaneously
    //pros::lcd::set_text(2, "6: Align loader");
    turnFor(AUTO_TURN_SPEED, AUTO_TURN_135_DEG, true);  // Turn right 135 degrees
    
    pros::delay(AUTO_SIMULTANEOUS_DELAY);
    deployScooper();
    
    pros::delay(200);

    if (!full) {
        pros::lcd::set_text(2, "=== END ===");
        return;
    }
    
    // STEP 7: Load blocks
    //pros::lcd::set_text(2, "7: Load blocks");
    driveFor(AUTO_DRIVE_SPEED, AUTO_DRIVE_TO_LOADER, true);
    
    pros::delay(AUTO_INTAKE_START_DELAY);
    outakeMiddle();
    
    pros::delay(AUTO_LOADER_TIME);
    stopIntake();
    
    // STEP 8: Exit & retract simultaneously
    //pros::lcd::set_text(2, "8: Exit loader");
    driveFor(AUTO_DRIVE_SPEED, AUTO_EXIT_LOADER, false);  // Reverse
    
    pros::delay(AUTO_SIMULTANEOUS_DELAY);
    retractScooper();
    
    pros::delay(200);
    
    // STEP 9: Turn to long goal
    //pros::lcd::set_text(2, "9: To long goal");
    turnFor(AUTO_TURN_SPEED, AUTO_TURN_180_DEG, false);  // Turn left 180 degrees
    
    // STEP 10: Score long goal
    //pros::lcd::set_text(2, "10: Score long");
    driveFor(AUTO_DRIVE_SPEED, AUTO_DRIVE_LONG_GOAL, true);
    
    outakeTop();
    //pros::delay(AUTO_LONG_GOAL_TIME);
    stopIntake();
    
    //pros::lcd::set_text(2, "=== COMPLETE ===");*/
}


void autonomousRight(bool full = true) {
    pros::lcd::set_text(1, "=== AUTO: RIGHT ===");
    
    // STEP 1: Retract & approach blocks
    //pros::lcd::set_text(2, "1: Approach");
    retractScooper();
    pros::delay(AUTO_PNEUMATIC_DELAY);
    
    // Drive forward to blocks
    driveFor(AUTO_DRIVE_SPEED, AUTO_DRIVE_TO_BLOCKS * 0.8, true);
    
    // STEP 2: Deploy & intake while moving
    //pros::lcd::set_text(2, "2: Intake");
    deployScooper();
    pros::delay(100);
    
    outakeMiddle();
    driveFor(AUTO_DRIVE_SPEED, AUTO_DRIVE_TO_BLOCKS * 0.2, true);
    
    pros::delay(AUTO_INTAKE_TIME);
    stopIntake();
    
    // STEP 3: Turn & retract simultaneously
    //pros::lcd::set_text(2, "3: Align");
    turnFor(AUTO_TURN_SPEED, AUTO_TURN_45_DEG, false);  // Turn LEFT 45 degrees
    
    pros::delay(AUTO_SIMULTANEOUS_DELAY);
    retractScooper();
    
    pros::delay(200);
    
    // STEP 4: Score center goal
    //pros::lcd::set_text(2, "4: Score center");
    driveFor(AUTO_DRIVE_SPEED, AUTO_APPROACH_GOAL, true);
    
    outakeBottom();
    pros::delay(AUTO_OUTTAKE_TIME);
    stopIntake();
    
    // STEP 5: Back to loader
    //pros::lcd::set_text(2, "5: To loader");
    driveFor(AUTO_DRIVE_SPEED, AUTO_BACK_TO_LOADER, false);  // Reverse
    
    // STEP 6: Turn & deploy simultaneously
    //pros::lcd::set_text(2, "6: Align loader");
    turnFor(AUTO_TURN_SPEED, AUTO_TURN_135_DEG, false);  // Turn LEFT 135 degrees
    
    pros::delay(AUTO_SIMULTANEOUS_DELAY);
    deployScooper();
    
    pros::delay(200);

    if (!full) {
        //pros::lcd::set_text(2, "=== END ===");
        return;
    }
    
    // STEP 7: Load blocks
    //pros::lcd::set_text(2, "7: Load blocks");
    driveFor(AUTO_DRIVE_SPEED, AUTO_DRIVE_TO_LOADER, true);
    
    pros::delay(AUTO_INTAKE_START_DELAY);
    outakeMiddle();
    
    pros::delay(AUTO_LOADER_TIME);
    stopIntake();
    
    // STEP 8: Exit & retract simultaneously
    //pros::lcd::set_text(2, "8: Exit loader");
    driveFor(AUTO_DRIVE_SPEED, AUTO_EXIT_LOADER, false);  // Reverse
    
    pros::delay(AUTO_SIMULTANEOUS_DELAY);
    retractScooper();
    
    pros::delay(200);
    
    // STEP 9: Turn to long goal
    //pros::lcd::set_text(2, "9: To long goal");
    turnFor(AUTO_TURN_SPEED, AUTO_TURN_180_DEG, true);  // Turn RIGHT 180 degrees
    
    // STEP 10: Score long goal
    //pros::lcd::set_text(2, "10: Score long");
    driveFor(AUTO_DRIVE_SPEED, AUTO_DRIVE_LONG_GOAL, true);
    
    outakeTop();
    pros::delay(AUTO_LONG_GOAL_TIME);
    stopIntake();
    
    //pros::lcd::set_text(2, "=== COMPLETE ===");
}


// ==================== MAIN AUTONOMOUS ====================

void autonomous() {
    autonomousMode = true;
    setDriveBrakeMode(pros::E_MOTOR_BRAKE_HOLD);
    
    pros::lcd::clear();
    
    // Uncomment ONE:
    autonomousLeft();
    //autonomousRight();

    autonomousMode = false;
    setDriveBrakeMode(pros::E_MOTOR_BRAKE_COAST);
}


// ==================== DRIVER CONTROL ====================

void opcontrol() {
    autonomousMode = false;
    setDriveBrakeMode(pros::E_MOTOR_BRAKE_COAST);
    
    while (true) {
        // Drivetrain
        int forward = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int turnLeft = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);
        int turnRight = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int turn = turnLeft + turnRight;
        
        omniDrive(forward, 0, turn);
        
        // Intake controls
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            outakeMiddle();
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            outakeTop();
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            outakeBottom();
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            outakeStorage();
        }
        else {
            stopIntake();
        }
        
        // Pneumatics
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_X)) {
            deployScooper();
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_B)) {
            retractScooper();
        }
        
        // Manual controls for specific motors
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            motor6CCW();
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_Y)) {
            motor6CW();
        }
        
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            motor8CW();
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_RIGHT)) {
            motor8CCW();
        }

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_UP)) {
            motor10CCW();
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN)) {
            motor10CW();
        }
        
        pros::delay(MAIN_LOOP_DELAY);
    }
}