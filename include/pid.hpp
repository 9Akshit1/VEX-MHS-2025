#ifndef PID_HPP
#define PID_HPP

/**
 * PID Controller Class
 * Implements a Proportional-Integral-Derivative controller for closed-loop control
 * 
 * Usage:
 * - Create PID object with kP, kI, kD constants
 * - Call update() with target and current values each loop iteration
 * - Use returned output to control motors/mechanisms
 * 
 * Features:
 * - Integral active zone to prevent windup
 * - Error reset function for switching between control loops
 * - Configurable gains for tuning
 */
class PID {
private:
    long double kP;           // Proportional gain
    long double kI;           // Integral gain
    long double kD;           // Derivative gain
    long double errorSum;     // Accumulated error for integral term
    long double lastError;    // Previous error for derivative term

public:
    /**
     * Constructor - Initialize PID controller with gains
     * 
     * @param proportional Proportional gain (kP)
     * @param integral Integral gain (kI)
     * @param derivative Derivative gain (kD)
     */
    PID(long double proportional, long double integral, long double derivative);

    /**
     * Default constructor - Initializes with zero gains
     */
    PID();

    /**
     * Reset accumulated errors
     * Call this when switching targets or disabling the controller
     * to prevent integral windup
     */
    void reset();

    /**
     * Update PID controller and calculate output
     * 
     * @param target The desired setpoint value
     * @param current The current measured value
     * @param integralZone The error threshold below which integral is active (prevents windup)
     * @return The calculated control output (P + I + D)
     */
    long double update(long double target, long double current, long double integralZone = 1000000.0);

    /**
     * Set PID gains (for tuning on-the-fly)
     */
    void setGains(long double proportional, long double integral, long double derivative);

    /**
     * Get current error value
     */
    long double getError() const { return lastError; }

    /**
     * Get accumulated error (integral term)
     */
    long double getErrorSum() const { return errorSum; }
};

#endif // PID_HPP