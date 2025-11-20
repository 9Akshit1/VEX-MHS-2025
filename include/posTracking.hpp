#ifndef POSTRACKING_HPP
#define POSTRACKING_HPP

#include "main.h"

/**
 * Odometry position tracking class
 * Calculates robot displacement using single vertical encoder + horizontal encoder + IMU
 * 
 * Mathematical explanation:
 * - Uses arc geometry to convert encoder changes into global coordinate changes
 * - Tracks change in heading using IMU sensor
 * - Calculates displacement along arc using chord length approximation
 * - Transforms local displacements to global coordinate frame
 */
class PositionTracking {
private:
    long double deltaAngle;      // Change in robot heading (radians)
    long double deltaV;          // Vertical tracking wheel displacement (inches)
    long double deltaB;          // Back/horizontal tracking wheel displacement (inches)
    long double halfAngle;       // Half of delta angle for calculations
    long double globalAngle;     // Global robot heading (radians)
    long double chordLength;     // Length of arc chord (forward/back displacement)
    long double chordLengthH;    // Length of horizontal chord (strafe displacement)
    long double xDisplacement;   // Global X displacement
    long double yDisplacement;   // Global Y displacement

public:
    /**
     * Calculate position change based on tracking wheel deltas and IMU heading
     * 
     * @param lastAngle Previous global heading (radians)
     * @param currentH Current horizontal encoder position (inches)
     * @param lastH Previous horizontal encoder position (inches)
     * @param currentV Current vertical encoder position (inches)
     * @param lastV Previous vertical encoder position (inches)
     * @param currentAngle Current IMU heading (radians)
     */
    PositionTracking(
        long double lastAngle,
        long double currentH, long double lastH,
        long double currentV, long double lastV,
        long double currentAngle
    );

    // Getters for calculated values
    long double getX() const { return xDisplacement; }
    long double getY() const { return yDisplacement; }
    long double getAngle() const { return globalAngle; }
};

/**
 * Motion planning utility class
 * Calculates angle and distance between two points
 */
class Motion {
private:
    long double targetAngle;   // Angle to target (radians)
    long double distance;      // Distance to target (inches)

public:
    /**
     * Calculate motion parameters from current to target position
     * 
     * @param currentX Current X position
     * @param currentY Current Y position
     * @param targetX Target X position
     * @param targetY Target Y position
     */
    Motion(long double currentX, long double currentY, long double targetX, long double targetY);

    long double getAngle() const { return targetAngle; }
    long double getDistance() const { return distance; }
};

/**
 * Odometry task function - runs continuously in background
 * Updates global position variables based on tracking wheel movements and IMU
 * Only runs if ODOMETRY_ENABLED is true
 * 
 * Hardware requirements:
 * - Single vertical tracking wheel (forward/backward movement)
 * - Horizontal tracking wheel (strafe movement)
 * - IMU sensor (heading/rotation)
 */
void odometryTask(void* param);

#endif // POSTRACKING_HPP