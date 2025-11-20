#ifndef GLOBALS_HPP
#define GLOBALS_HPP

#include "main.h"
#include <array>
#include "pid.hpp"

// ==================== CONFIGURATION ====================
// Set this to true if you have odometry installed, false otherwise
extern const bool ODOMETRY_ENABLED;

// ==================== CONTROLLER ====================
extern pros::Controller master;

// ==================== MOTORS ====================
// Drive motors (update ports to match your robot)
extern pros::Motor leftFront;
extern pros::Motor leftBack;
extern pros::Motor rightFront;
extern pros::Motor rightBack;

// Mechanism motors (update/add/remove based on your robot)
extern pros::Motor leftIntake;
extern pros::Motor rightIntake;
extern pros::Motor indexer;
extern pros::Motor shooter;

// ==================== SENSORS ====================
// IMU (required for odometry, recommended regardless)
extern pros::Imu inertial;

// Odometry tracking wheels (only used if ODOMETRY_ENABLED = true)
extern pros::adi::Encoder verticalEncoder1;
extern pros::adi::Encoder horizontalEncoder;

// Line trackers for ball detection (optional, based on your mechanism)
extern pros::adi::AnalogIn line_tracker1;
extern pros::adi::AnalogIn line_tracker2;

// ==================== CONSTANTS ====================
// Mathematical constants
extern const long double PI;
extern const long double EPS;

// Drive constants
extern const int MAX_VOLTAGE;

// Odometry constants (only used if ODOMETRY_ENABLED = true)
extern const long double VERTICAL_OFFSET_LEFT;   // Distance from center to left tracking wheel (inches)
extern const long double HORIZONTAL_OFFSET;      // Distance from center to back tracking wheel (inches)
extern const long double WHEEL_DIAMETER;         // Tracking wheel diameter (inches)
extern const long double TICKS_PER_REV;          // Encoder ticks per revolution
extern const long double TICKS_TO_INCHES;        // Conversion factor

// IMU sensor fusion weights (only used if ODOMETRY_ENABLED = true)
extern const long double TRACKING_WEIGHT;
extern const long double IMU_WEIGHT;
extern const long double IMU_SCALING;

// Indexing constants (optional, based on your mechanism)
extern const int INDEX_THRESHOLD;
extern const int INDEX_TIME_MS;

// PID tuning values for different subsystems
// Format: {kP, kI, kD}
extern const long double DRIVE_PID_KP;
extern const long double DRIVE_PID_KI;
extern const long double DRIVE_PID_KD;

extern const long double TURN_PID_KP;
extern const long double TURN_PID_KI;
extern const long double TURN_PID_KD;

extern const long double STRAFE_PID_KP;
extern const long double STRAFE_PID_KI;
extern const long double STRAFE_PID_KD;

// ==================== GLOBAL VARIABLES ====================
// Odometry global position (only used if ODOMETRY_ENABLED = true)
extern long double globalX;
extern long double globalY;
extern long double globalAngle;

// Mutex for thread-safe odometry access
extern pros::Mutex odom_mutex;

// ==================== AUTONOMOUS FUNCTIONS ====================
// Declare autonomous helper functions
void deploy();
void ballDistribution(int ballsToShoot, int ballsToGrab);
bool translationPID(long double targetX, long double targetY, long double targetHeading, 
                    uint32_t timeout, bool runIntake, int maxVoltage);
bool pointTurnPID(long double targetAngle, uint32_t timeout);

#endif // GLOBALS_HPP