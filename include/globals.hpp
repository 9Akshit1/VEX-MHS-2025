/**
 * globals.hpp - Global declarations
 * 
 * This file contains extern declarations for global objects.
 * The actual definitions are in main.cpp
 */

#ifndef GLOBALS_HPP
#define GLOBALS_HPP

#include "main.h"
#include "lemlib/api.hpp"

// ==================== EXTERN DECLARATIONS ====================
// These are DECLARED here but DEFINED in main.cpp

// Motors
extern pros::Motor leftIntake;
extern pros::Motor rightIntake;
extern pros::Motor indexer;
extern pros::Motor shooter;

// Controller
extern pros::Controller master;

// Sensors
extern pros::adi::AnalogIn line_tracker1;
extern pros::adi::AnalogIn line_tracker2;

// Chassis (for accessing odometry in other files)
extern lemlib::Chassis chassis;

// Constants
extern const float PI;
extern const int INDEX_THRESHOLD;

// Ball tracking state
extern int ballsTaken;
extern int ballsShot;
extern bool ballCountingActive;

// Helper functions
void ballCountingTask(void* param);
void deploy();
void ballDistribution(int ballsToShoot, int ballsToGrab);

#endif // GLOBALS_HPP