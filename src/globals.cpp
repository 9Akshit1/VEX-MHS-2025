#include "globals.hpp"

// ==================== CONFIGURATION ====================
// *** SET THIS TO true WHEN YOU INSTALL ODOMETRY, false OTHERWISE ***
const bool ODOMETRY_ENABLED = true;

// ==================== CONTROLLER ====================
pros::Controller master(pros::E_CONTROLLER_MASTER);

// ==================== MOTORS ====================
// *** UPDATE THESE PORT NUMBERS TO MATCH YOUR ROBOT ***
// Use negative numbers for reversed motors
// Change motor cartridge type as needed: blue, green, red
pros::Motor leftFront(1, pros::MotorCartridge::green);    // Port 1, green cartridge, not reversed
pros::Motor leftBack(2, pros::MotorCartridge::green);    // Port 2, green cartridge, not reversed
pros::Motor rightFront(-3, pros::MotorCartridge::green);    // Port 3, green cartridge, REVERSED
pros::Motor rightBack(-4, pros::MotorCartridge::green);    // Port 4, green cartridge, REVERSED

// *** UPDATE/ADD/REMOVE MECHANISM MOTORS BASED ON YOUR ROBOT ***
pros::Motor leftIntake(5, pros::MotorCartridge::green);    // Port 5, green cartridge, not reversed
pros::Motor rightIntake(-6, pros::MotorCartridge::green);    // Port 6, green cartridge, REVERSED
pros::Motor indexer(7, pros::MotorCartridge::green);    // Port 7, green cartridge, not reversed
pros::Motor shooter(8, pros::MotorCartridge::green);    // Port 8, green cartridge, not reversed

// ==================== SENSORS ====================
// *** UPDATE IMU PORT ***
pros::Imu inertial(9);

// *** UPDATE ENCODER PORTS (only matters if ODOMETRY_ENABLED = true) ***
// Format: port1, port2, reversed?
pros::adi::Encoder verticalEncoder1('A', 'B', false);
pros::adi::Encoder horizontalEncoder('E', 'F', false);

// *** UPDATE LINE TRACKER PORTS (optional) ***
pros::adi::AnalogIn line_tracker1('G');
pros::adi::AnalogIn line_tracker2('H');

// ==================== CONSTANTS ====================
const long double PI = 3.14159265358979323846;
const long double EPS = 1e-8;
const int MAX_VOLTAGE = 12000;

// *** ODOMETRY CALIBRATION VALUES (measure these for your robot) ***
const long double VERTICAL_OFFSET_LEFT = 7.125;    // inches from robot center
const long double HORIZONTAL_OFFSET = 7.45;        // inches from robot center
const long double WHEEL_DIAMETER = 2.75;           // inches
const long double TICKS_PER_REV = 360.0;           // encoder ticks per wheel revolution
const long double TICKS_TO_INCHES = (PI * WHEEL_DIAMETER) / TICKS_PER_REV;

// *** IMU SENSOR FUSION TUNING ***
const long double TRACKING_WEIGHT = 0.01;  // Weight for tracking wheels (0-1)
const long double IMU_WEIGHT = 0.99;       // Weight for IMU (0-1), should sum to 1.0 with TRACKING_WEIGHT
const long double IMU_SCALING = 0.9975;    // IMU calibration coefficient

// *** INDEXING TUNING (optional) ***
const int INDEX_THRESHOLD = 2800;  // Line sensor threshold for ball detection
const int INDEX_TIME_MS = 140;     // Time to run indexer when ball detected

// *** PID CONSTANTS *** -----------------------------------------   TUNE THESE VALUES FOR YOUR ROBOT
// Drive PID (forward/backward movement)
const long double DRIVE_PID_KP = 52.0;
const long double DRIVE_PID_KI = 2.0;
const long double DRIVE_PID_KD = 300.0;

// Turn PID (rotation)
const long double TURN_PID_KP = 14000.0;
const long double TURN_PID_KI = 2.0;
const long double TURN_PID_KD = 180.0;

// Strafe PID (sideways movement for X-drive)
const long double STRAFE_PID_KP = 52.0;
const long double STRAFE_PID_KI = 2.0;
const long double STRAFE_PID_KD = 180.0;

// ==================== GLOBAL VARIABLES ====================
long double globalX = 0.0;
long double globalY = 0.0;
long double globalAngle = 0.0;

pros::Mutex odom_mutex;