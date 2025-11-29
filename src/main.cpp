/**
 * main.cpp - LemLib WITHOUT IMU (Fixed Configuration)
 * 
 * Encoders work but LemLib returning NaN = configuration issue
 * This version uses simpler, more robust tracking wheel setup
 */

#include "main.h"
#include "lemlib/api.hpp"

// ==================== DRIVETRAIN MOTORS ====================
pros::MotorGroup leftMotors({1, 2}, pros::MotorGearset::green);
pros::MotorGroup rightMotors({-3, -4}, pros::MotorGearset::green);

// ==================== SENSORS ====================
pros::adi::Encoder verticalEncoder('A', 'B', false);
pros::adi::Encoder horizontalEncoder('E', 'F', false);

// Line sensors
pros::adi::AnalogIn line_tracker1('G');
pros::adi::AnalogIn line_tracker2('H');

// ==================== MECHANISM MOTORS ====================
pros::Motor leftIntake(5, pros::MotorGearset::green);
pros::Motor rightIntake(-6, pros::MotorGearset::green);
pros::Motor indexer(7, pros::MotorGearset::green);
pros::Motor shooter(8, pros::MotorGearset::green);

// ==================== CONTROLLER ====================
pros::Controller master(pros::E_CONTROLLER_MASTER);

// ==================== CONSTANTS ====================
const int INDEX_THRESHOLD = 2800;

// ==================== ROBOT MEASUREMENTS ====================
// CRITICAL: Measure these values on your actual robot!

// Tracking wheel diameter in inches (usually 2.75" for omni wheels)
const float WHEEL_DIAMETER = 5.0;

// Horizontal distance between left and right drive wheels (track width)
const float TRACK_WIDTH = 11.0;

// Distance from tracking center to vertical encoder (perpendicular tracking wheel)
// This is how far forward/back the perpendicular wheel is from robot center
const float VERTICAL_OFFSET = 1.75;  // Start with 0, tune if turns drift

// Distance from tracking center to horizontal encoder (parallel tracking wheel)  
// This is how far left/right the parallel wheel is from robot center
const float HORIZONTAL_OFFSET = 5.5;  // Start with 0, tune if strafing drifts

// ==================== LEMLIB CONFIGURATION ====================

// Create tracking wheels with simplified configuration
// Vertical wheel tracks forward/backward movement
lemlib::TrackingWheel vertical(
    &verticalEncoder,
    2.75,  // wheel diameter in inches
    7.125, // distance from center (your measurement)
    1      // gear ratio
);

lemlib::TrackingWheel horizontal(
    &horizontalEncoder,
    2.75,
    -7.45,  // negative for left side
    1
);

// Lateral PID (forward/backward movement)
lemlib::ControllerSettings lateralController(
    8,     // kP - proportional gain
    0,     // kI - integral gain (start at 0)
    40,    // kD - derivative gain
    0,     // anti-windup (not used when kI=0)
    1,     // small error range (inches)
    100,   // small error timeout (ms)
    3,     // large error range (inches)
    500,   // large error timeout (ms)
    20     // max acceleration
);

// Angular PID (turning)
lemlib::ControllerSettings angularController(
    4,     // kP - proportional gain
    0,     // kI - integral gain (start at 0)
    40,    // kD - derivative gain
    0,     // anti-windup
    1,     // small error range (degrees)
    100,   // small error timeout (ms)
    3,     // large error range (degrees)
    500,   // large error timeout (ms)
    0      // max acceleration (0 for turning)
);

// Drivetrain configuration
lemlib::Drivetrain drivetrain(
    &leftMotors,
    &rightMotors,
    TRACK_WIDTH,
    lemlib::Omniwheel::NEW_275,  // 2.75" drive wheels
    360,  // RPM for blue cartridge (200=green, 360=blue, 600=red)
    2     // horizontal drift multiplier
);

// Odometry sensors - CRITICAL: Must have NO IMU for 2-wheel odometry
lemlib::OdomSensors sensors(
    nullptr,        // vertical tracking wheel 1 - SET TO NULL
    nullptr,        // vertical tracking wheel 2
    nullptr,        // horizontal tracking wheel 1 - SET TO NULL
    nullptr,        // horizontal tracking wheel 2
    nullptr         // NO IMU
);

// Create chassis
lemlib::Chassis chassis(drivetrain, lateralController, angularController, sensors);

// Manual odometry update task since LemLib 2-wheel is broken
void odometryTask(void* param) {
    float lastVertical = 0;
    float lastHorizontal = 0;
    float x = 0;
    float y = 0;
    float heading = 0;
    
    while (true) {
        // Get current distances
        float currentVertical = vertical.getDistanceTraveled();
        float currentHorizontal = horizontal.getDistanceTraveled();
        
        // Calculate deltas
        float deltaVertical = currentVertical - lastVertical;
        float deltaHorizontal = currentHorizontal - lastHorizontal;
        
        // Update heading based on horizontal movement
        heading += deltaHorizontal / TRACK_WIDTH;
        
        // Update position
        x += deltaVertical * cos(heading);
        y += deltaVertical * sin(heading);
        
        // Update chassis pose
        chassis.setPose(x, y, heading * 180.0 / 3.14159);
        
        // Save for next loop
        lastVertical = currentVertical;
        lastHorizontal = currentHorizontal;
        
        pros::delay(10);
    }
}

// ==================== BALL TRACKING ====================
int ballsTaken = 0;
int ballsShot = 0;
bool bottomSensorCovered = true;
bool topSensorCovered = true;
bool ballCountingActive = false;

void ballCountingTask(void* param) {
    while (true) {
        if (ballCountingActive) {
            if (line_tracker1.get_value() >= INDEX_THRESHOLD && bottomSensorCovered) {
                bottomSensorCovered = false;
            } else if (line_tracker1.get_value() < INDEX_THRESHOLD && !bottomSensorCovered) {
                ballsTaken++;
                bottomSensorCovered = true;
            }

            if (line_tracker2.get_value() >= INDEX_THRESHOLD && topSensorCovered) {
                ballsShot++;
                topSensorCovered = false;
            } else if (line_tracker2.get_value() < INDEX_THRESHOLD && !topSensorCovered) {
                topSensorCovered = true;
            }
        }
        pros::delay(10);
    }
}

void deploy() {
    leftIntake.move_velocity(-200);
    rightIntake.move_velocity(200);
    indexer.move_velocity(200);
    pros::delay(250);
    indexer.move_velocity(0);
    pros::delay(250);
    leftIntake.move_velocity(0);
    rightIntake.move_velocity(0);
}

void ballDistribution(int ballsToShoot, int ballsToGrab) {
    ballsShot = 0;
    ballsTaken = 0;
    ballCountingActive = true;

    uint32_t indexStartTime = pros::millis();
    while (line_tracker2.get_value() >= INDEX_THRESHOLD && 
           (pros::millis() - indexStartTime) < 1800) {
        indexer.move_velocity(-200);
        shooter.move_velocity(190);
    }

    indexer.move_velocity(-110);
    leftIntake.move_velocity(-200);
    rightIntake.move_velocity(200);
    shooter.move_velocity(190);

    while (true) {
        if (ballsTaken >= ballsToGrab && ballsShot >= ballsToShoot) {
            break;
        } else if (ballsTaken >= ballsToGrab) {
            indexer.move_velocity(0);
            leftIntake.move_velocity(0);
            rightIntake.move_velocity(0);
        } else if (ballsShot >= ballsToShoot) {
            shooter.move_velocity(0);
            shooter.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
        }
        pros::delay(10);
    }

    indexer.move_velocity(0);
    leftIntake.move_velocity(0);
    rightIntake.move_velocity(0);
    shooter.move_velocity(0);
    ballCountingActive = false;
}

// ==================== INITIALIZATION ====================
void initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "=== MINIMAL TEST ===");
    
    // Reset encoders
    verticalEncoder.reset();
    horizontalEncoder.reset();
    pros::delay(200);
    
    // DON'T use chassis at all - manually set position
    float x = 0;
    float y = 0;
    float h = 0;
    
    pros::lcd::set_text(2, "X:" + std::to_string(x));
    pros::lcd::set_text(3, "Y:" + std::to_string(y));
    pros::lcd::set_text(4, "H:" + std::to_string(h));
    
    // Try to manually call setPose
    chassis.setPose(5.0, 10.0, 45.0);
    pros::delay(500);
    
    lemlib::Pose test = chassis.getPose();
    pros::lcd::set_text(5, "After setPose:");
    pros::lcd::set_text(6, "X:" + std::to_string(test.x));
    pros::lcd::set_text(7, "Y:" + std::to_string(test.y));
    
    pros::delay(5000);
}

void old_initialize() {
    pros::lcd::initialize();
    pros::lcd::set_text(1, "Initializing...");
    
    // STEP 1: Reset encoders to zero
    pros::lcd::set_text(2, "Resetting encoders...");
    verticalEncoder.reset();
    horizontalEncoder.reset();
    pros::delay(100);
    
    // STEP 2: Verify encoders reset
    int v_initial = verticalEncoder.get_value();
    int h_initial = horizontalEncoder.get_value();
    pros::lcd::set_text(3, "V:" + std::to_string(v_initial) + " H:" + std::to_string(h_initial));
    
    // Check if encoders are accessible
    if (v_initial == PROS_ERR || h_initial == PROS_ERR) {
        pros::lcd::set_text(4, "ERROR: Encoder ports!");
        pros::lcd::set_text(5, "Check A,B and E,F");
        return;
    }
    
    pros::delay(500);
    
    // STEP 3: Initialize chassis (DO NOT call calibrate for 2-wheel odom without IMU)
    pros::lcd::set_text(4, "Initializing chassis...");
    
    // For 2-wheel odometry without IMU, we ONLY set the initial pose
    // We do NOT call chassis.calibrate() as it expects an IMU
    chassis.calibrate(false);  // false = don't calibrate IMU
    pros::delay(500);
    chassis.setPose(0, 0, 0);
    // DEBUGGING - Check what values LemLib is seeing
    pros::delay(500);

    pros::Task odomTask(odometryTask, nullptr, "Odometry");

    // Test 1: Can we read encoders?
    int v_test = verticalEncoder.get_value();
    int h_test = horizontalEncoder.get_value();
    pros::lcd::set_text(4, "EncTest V:" + std::to_string(v_test) + " H:" + std::to_string(h_test));

    // Test 2: Can tracking wheels read encoders?
    int v_wheel = vertical.getDistanceTraveled();
    int h_wheel = horizontal.getDistanceTraveled();
    pros::lcd::set_text(5, "Wheels V:" + std::to_string(v_wheel) + " H:" + std::to_string(h_wheel));

    // Test 3: What does getPose return?
    lemlib::Pose testPose = chassis.getPose();
    pros::lcd::set_text(6, "Pose X:" + std::to_string(testPose.x));
    pros::lcd::set_text(7, "Pose Y:" + std::to_string(testPose.y));
    pros::lcd::set_text(8, "Pose H:" + std::to_string(testPose.theta));

    pros::delay(5000);  // Give yourself time to read it
    
    if (std::isnan(testPose.x) || std::isnan(testPose.y) || std::isnan(testPose.theta)) {
        pros::lcd::set_text(8, "STILL NaN - LemLib issue");
    } else {
        pros::lcd::set_text(8, "SUCCESS! Ready to go!");
    }
    
    pros::delay(2000);
}

void disabled() {
    leftIntake.move(0);
    rightIntake.move(0);
    indexer.move(0);
    shooter.move(0);
}

void competition_initialize() {}

// ==================== AUTONOMOUS ====================
void autonomous() {
    pros::lcd::clear();
    pros::lcd::set_text(1, "=== AUTONOMOUS ===");
    
    // Verify odometry
    lemlib::Pose startPose = chassis.getPose();
    if (std::isnan(startPose.x) || std::isnan(startPose.y)) {
        pros::lcd::set_text(2, "ERROR: Odometry NaN");
        pros::lcd::set_text(3, "Cannot run autonomous");
        return;
    }
    
    pros::lcd::set_text(2, "Odometry OK - Running");
    
    // Start ball counter
    pros::Task ballCounter(ballCountingTask, nullptr, "Ball Counter");

    // Deploy
    deploy();
    pros::delay(500);

    // Move forward 24 inches
    pros::lcd::set_text(3, "Forward 24in...");
    chassis.moveToPose(0, 24, 0, 3000, {.minSpeed = 60});
    pros::delay(200);

    // Turn 90 degrees right
    pros::lcd::set_text(3, "Turn 90deg...");
    chassis.turnToHeading(90, 2000, {.minSpeed = 30});
    pros::delay(200);

    // Move to (24, 24)
    pros::lcd::set_text(3, "To (24,24)...");
    chassis.moveToPose(24, 24, 90, 3000, {.minSpeed = 60});
    pros::delay(200);

    // Shoot and grab balls
    pros::lcd::set_text(3, "Ball routine...");
    ballDistribution(2, 1);
    pros::delay(200);

    // Turn toward origin
    pros::lcd::set_text(3, "Turn to origin...");
    chassis.turnToPoint(0, 0, 2000, {.minSpeed = 30});
    pros::delay(200);

    // Return to origin
    pros::lcd::set_text(3, "Return home...");
    chassis.moveToPose(0, 0, 180, 3000, {.forwards = false, .minSpeed = 60});

    pros::lcd::set_text(1, "=== COMPLETE ===");
    lemlib::Pose final = chassis.getPose();
    pros::lcd::set_text(2, "X:" + std::to_string(final.x).substr(0, 5));
    pros::lcd::set_text(3, "Y:" + std::to_string(final.y).substr(0, 5));
}

// ==================== DRIVER CONTROL ====================
void opcontrol() {
    bool autoIndexActive = false;
    uint32_t autoIndexStartTime = 0;

    // Display task
    pros::Task screenTask([&]() {
        while (true) {
            int vertVal = verticalEncoder.get_value();
            int horizVal = horizontalEncoder.get_value();
            lemlib::Pose pose = chassis.getPose();

            if (std::isnan(pose.x) || std::isnan(pose.y) || std::isnan(pose.theta)) {
                pros::lcd::set_text(1, "ERROR: Position NaN");
                pros::lcd::set_text(2, "EncV: " + std::to_string(vertVal));
                pros::lcd::set_text(3, "EncH: " + std::to_string(horizVal));
            } else {
                pros::lcd::set_text(1, "X: " + std::to_string(pose.x).substr(0, 6));
                pros::lcd::set_text(2, "Y: " + std::to_string(pose.y).substr(0, 6));
                pros::lcd::set_text(3, "H: " + std::to_string(pose.theta).substr(0, 6));
                pros::lcd::set_text(4, "EncV: " + std::to_string(vertVal));
                pros::lcd::set_text(5, "EncH: " + std::to_string(horizVal));
            }

            pros::delay(100);
        }
    });

    while (true) {
        // Tank drive
        int left = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int right = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        chassis.tank(left, right);

        // Auto-indexing
        if (autoIndexActive && 
            (pros::millis() - autoIndexStartTime) < 140 && 
            line_tracker2.get_value() >= INDEX_THRESHOLD) {
            indexer.move_velocity(-200);
            shooter.move_velocity(160);
        } else {
            autoIndexActive = false;
        }

        // Intake control
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            leftIntake.move_velocity(-200);
            rightIntake.move_velocity(200);

            if (line_tracker2.get_value() < INDEX_THRESHOLD) {
                shooter.move_velocity(0);
                indexer.move_velocity(0);
                shooter.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
                autoIndexActive = false;
            }

            if (line_tracker1.get_value() < INDEX_THRESHOLD && !autoIndexActive) {
                autoIndexActive = true;
                autoIndexStartTime = pros::millis();
            }
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            leftIntake.move_velocity(200);
            rightIntake.move_velocity(-200);
        } else {
            leftIntake.move_velocity(0);
            rightIntake.move_velocity(0);
        }

        // Shooter control
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && 
            master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            indexer.move_velocity(-150);
            shooter.move_velocity(-200);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            indexer.move_velocity(-200);
            shooter.move_velocity(200);
            shooter.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            shooter.move_velocity(-200);
            indexer.move_velocity(200);
            shooter.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        } else if (!autoIndexActive) {
            shooter.move_velocity(0);
            indexer.move_velocity(0);
        }

        // Test buttons
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            master.rumble("-");
            lemlib::Pose current = chassis.getPose();
            
            if (!std::isnan(current.y)) {
                master.print(0, 0, "Forward 24in");
                chassis.moveToPose(current.x, current.y + 24, current.theta, 3000, {.minSpeed = 60});
                master.rumble(".");
            } else {
                master.print(0, 0, "ERR: NaN");
                master.rumble("---");
            }
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_B)) {
            master.rumble("-");
            lemlib::Pose current = chassis.getPose();
            
            if (!std::isnan(current.theta)) {
                master.print(0, 0, "Turn 90");
                chassis.turnToHeading(current.theta + 90, 2000, {.minSpeed = 30});
                master.rumble(".");
            } else {
                master.print(0, 0, "ERR: NaN");
                master.rumble("---");
            }
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
            chassis.setPose(0, 0, 0);
            master.rumble(".");
            master.print(0, 0, "Reset");
        }

        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {
            master.rumble("-");
            master.print(0, 0, "Recal...");
            
            verticalEncoder.reset();
            horizontalEncoder.reset();
            pros::delay(200);
            
            chassis.setPose(0, 0, 0);
            pros::delay(200);
            
            master.rumble("..");
            master.print(0, 0, "Done");
        }

        pros::delay(10);
    }
}