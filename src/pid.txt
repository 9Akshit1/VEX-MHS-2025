#include "pid.hpp"
#include <cmath>

/**
 * Constructor with specified gains
 */
PID::PID(long double proportional, long double integral, long double derivative)
    : kP(proportional), kI(integral), kD(derivative), errorSum(0.0), lastError(0.0) {
}

/**
 * Default constructor - zero gains
 */
PID::PID() : kP(0.0), kI(0.0), kD(0.0), errorSum(0.0), lastError(0.0) {
}

/**
 * Reset all error values
 * Call this when switching control loops or when PID is inactive
 */
void PID::reset() {
    errorSum = 0.0;
    lastError = 0.0;
}

/**
 * Update PID controller
 * 
 * How it works:
 * - Proportional: Responds to current error (how far off we are)
 * - Integral: Responds to accumulated error (corrects steady-state error)
 * - Derivative: Responds to rate of change (dampens oscillation)
 * 
 * Integral Active Zone:
 * - Only accumulates error when within this threshold
 * - Prevents "integral windup" when far from target
 * - Default is very large (essentially always active)
 */
long double PID::update(long double target, long double current, long double integralZone) {
    // Calculate current error
    long double error = target - current;
    
    // Calculate derivative (rate of change of error)
    long double errorDerivative = error - lastError;
    
    // Only accumulate integral if within active zone
    if (std::abs(error) < integralZone) {
        errorSum += error;
    } else {
        errorSum = 0.0; // Reset integral when outside zone
    }
    
    // Update last error for next iteration
    lastError = error;
    
    // Calculate each term
    long double proportional = kP * error;
    long double integral = kI * errorSum;
    long double derivative = kD * errorDerivative;
    
    // Return total output
    return proportional + integral + derivative;
}

/**
 * Set new PID gains
 * Useful for tuning or switching between aggressive/conservative control
 */
void PID::setGains(long double proportional, long double integral, long double derivative) {
    kP = proportional;
    kI = integral;
    kD = derivative;
}