#include "main.h"
#include "globals.hpp"
#include "posTracking.hpp"

/**
 * Operator control function
 * Runs during driver control period
 * 
 * Controls:
 * - Right stick Y: Forward/backward
 * - Right stick X: Strafe left/right
 * - Left stick X: Rotate left/right
 * - L1: Intake in
 * - L2: Intake out
 * - R1: Shoot/index
 * - R2: Reverse shooter
 * - A: Calibrate IMU
 */
void opcontrol() {
    // Auto-indexing state
    bool autoIndexActive = false;
    uint32_t autoIndexStartTime = 0;

    autonomous();   // simply for testing, but comment out during actual competition

    // Main control loop
    while (true) {
        // ==================== X-DRIVE CONTROL ====================
        // Get joystick inputs (range: -127 to 127)
        int forward = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);
        int strafe = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        int turn = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X);

        // Calculate motor powers for holonomic X-drive
        // Front-left: forward + strafe + turn
        // Back-left: forward - strafe + turn
        // Front-right: forward - strafe - turn
        // Back-right: forward + strafe - turn
        leftFront.move(forward + strafe + turn);
        leftBack.move(forward - strafe + turn);
        rightFront.move(forward - strafe - turn);
        rightBack.move(forward + strafe - turn);

        // ==================== AUTO-INDEXING LOGIC ====================
        // If auto-indexing is active and time hasn't elapsed
        if (autoIndexActive && 
            (pros::millis() - autoIndexStartTime) < INDEX_TIME_MS && 
            line_tracker2.get_value() >= INDEX_THRESHOLD) {
            indexer.move_velocity(-200);
            shooter.move_velocity(160);
        } else {
            autoIndexActive = false;
        }

        // ==================== INTAKE CONTROLS ====================
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            // Intake balls
            leftIntake.move_velocity(-200);
            rightIntake.move_velocity(200);

            // Stop indexer if ball is already at top
            if (line_tracker2.get_value() < INDEX_THRESHOLD) {
                shooter.move_velocity(0);
                indexer.move_velocity(0);
                shooter.set_brake_mode(pros::E_MOTOR_BRAKE_HOLD);
                autoIndexActive = false;
            }

            // Trigger auto-indexing when ball detected at intake
            if (line_tracker1.get_value() < INDEX_THRESHOLD && !autoIndexActive) {
                autoIndexActive = true;
                autoIndexStartTime = pros::millis();
            }
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            // Reverse intake
            leftIntake.move_velocity(200);
            rightIntake.move_velocity(-200);
        } else {
            // Stop intake
            leftIntake.move_velocity(0);
            rightIntake.move_velocity(0);
        }

        // ==================== SHOOTER/INDEXER CONTROLS ====================
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1) && 
            master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            // Both buttons: reverse both
            indexer.move_velocity(-150);
            shooter.move_velocity(-200);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            // R1: shoot/index forward
            indexer.move_velocity(-200);
            shooter.move_velocity(200);
            shooter.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            // R2: reverse shooter
            shooter.move_velocity(-200);
            indexer.move_velocity(200);
            shooter.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
        } else if (!autoIndexActive) {
            // Stop if not auto-indexing
            shooter.move_velocity(0);
            indexer.move_velocity(0);
        }

        // ==================== IMU CALIBRATION ====================
        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_A)) {
            pros::lcd::set_text(7, "Calibrating IMU...");
            inertial.reset();
            
            while (inertial.is_calibrating()) {
                pros::delay(10);
            }
            
            pros::lcd::set_text(7, "IMU calibrated!");
            
            // Reset odometry angle if enabled
            if (ODOMETRY_ENABLED) {
                odom_mutex.take(TIMEOUT_MAX);
                globalAngle = 0;
                odom_mutex.give();
            }
            
            pros::delay(500); // Debounce
        }

        // ==================== DISPLAY ODOMETRY (if enabled) ====================
        if (ODOMETRY_ENABLED) {
            odom_mutex.take(TIMEOUT_MAX);
            long double x = globalX;
            long double y = globalY;
            long double angle = globalAngle;
            odom_mutex.give();

            pros::lcd::set_text(5, "X: " + std::to_string(x));
            pros::lcd::set_text(6, "Y: " + std::to_string(y));
            pros::lcd::set_text(7, "Angle: " + std::to_string(angle * 180.0 / PI) + "°");
        } else {
            pros::lcd::set_text(5, "Odometry: OFF");
        }

        // Loop at 50Hz
        pros::delay(20);
    }
}