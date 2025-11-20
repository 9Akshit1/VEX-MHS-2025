#include "posTracking.hpp"
#include "globals.hpp"
#include <cmath>

// UPDATED HARDWARE SETUP:
// - Single vertical tracking wheel (forward/backward)
// - Horizontal tracking wheel (strafe)
// - IMU for heading

PositionTracking::PositionTracking(
    long double lastAngle,
    long double currentH, long double lastH,
    long double currentV, long double lastV,
    long double currentAngle
) : xDisplacement(0), yDisplacement(0) {
    
    // Calculate wheel displacements
    deltaB = currentH - lastH;  // Horizontal (strafe) movement
    deltaV = currentV - lastV;  // Vertical (forward/backward) movement

    // Use IMU for heading change
    deltaAngle = currentAngle - lastAngle;

    if (std::abs(deltaAngle) > EPS) {
        // Robot turned - use arc geometry
        halfAngle = deltaAngle / 2.0;
        
        // Calculate chord lengths using 2 * sin(θ/2) * radius
        // For vertical movement: radius = (deltaV / deltaAngle) + offset
        chordLength = 2.0 * std::sin(halfAngle) * ((deltaV / deltaAngle) + VERTICAL_OFFSET_LEFT);
        
        // For horizontal movement: radius = (deltaB / deltaAngle) + offset
        chordLengthH = 2.0 * std::sin(halfAngle) * ((deltaB / deltaAngle) + HORIZONTAL_OFFSET);
    } else {
        // Robot moved straight - no rotation, chords equal displacements
        chordLength = deltaV;
        chordLengthH = deltaB;
        halfAngle = 0;
    }

    // Convert local displacements to global coordinate frame
    globalAngle = lastAngle + halfAngle;
    
    // Forward/backward displacement components
    yDisplacement += chordLength * std::cos(globalAngle);
    xDisplacement += chordLength * std::sin(globalAngle);
    
    // Strafe displacement components (perpendicular to forward)
    yDisplacement += chordLengthH * (-std::sin(globalAngle));
    xDisplacement += chordLengthH * std::cos(globalAngle);
    
    // Update global angle
    globalAngle += halfAngle;
}

Motion::Motion(long double currentX, long double currentY, long double targetX, long double targetY) {
    long double dx = targetX - currentX;
    long double dy = targetY - currentY;

    // Calculate angle using atan2 (handles all quadrants correctly)
    targetAngle = std::atan2(dy, dx);
    
    // Calculate Euclidean distance
    distance = std::sqrt(dx * dx + dy * dy);
}

/**
 * Odometry background task
 * Continuously updates robot position based on tracking wheel movements
 */
void odometryTask(void* param) {
    if (!ODOMETRY_ENABLED) {
        pros::lcd::set_text(1, "Odometry: DISABLED");
        return;
    }

    // Wait for IMU to calibrate
    pros::delay(3000);

    // Initialize tracking variables
    long double lastV = 0, currentV = 0;
    long double lastH = 0, currentH = 0;
    long double lastAngle = 0, currentAngle = 0;

    pros::lcd::set_text(1, "Odometry: ACTIVE (1 Vert + Horiz)");

    while (true) {
        // Read current encoder positions and convert to inches
        currentV = verticalEncoder1.get_value() * TICKS_TO_INCHES;
        currentH = horizontalEncoder.get_value() * TICKS_TO_INCHES;
        
        // Get IMU heading in radians
        currentAngle = inertial.get_rotation() * (PI / 180.0);

        // Create position tracking instance to calculate displacement
        PositionTracking robotPos(
            lastAngle,
            currentH, lastH,
            currentV, lastV,
            currentAngle
        );

        // Update global position (thread-safe)
        if (!std::isnan(robotPos.getX()) && !std::isnan(robotPos.getY())) {
            odom_mutex.take(TIMEOUT_MAX);
            globalX += robotPos.getX();
            globalY += robotPos.getY();
            globalAngle = currentAngle;  // Use IMU directly for heading
            odom_mutex.give();
        }

        // Update last values
        lastAngle = currentAngle;
        lastV = currentV;
        lastH = currentH;

        // Display position on brain
        pros::lcd::set_text(2, "X: " + std::to_string(globalX));
        pros::lcd::set_text(3, "Y: " + std::to_string(globalY));
        pros::lcd::set_text(4, "Angle: " + std::to_string(globalAngle * 180.0 / PI));

        // Run at 50Hz (20ms loop time)
        pros::delay(20);
    }
}