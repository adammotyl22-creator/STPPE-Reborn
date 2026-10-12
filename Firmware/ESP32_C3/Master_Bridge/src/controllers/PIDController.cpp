#include "controllers/PIDController.h"
#include <cmath>

// Constructor implementation
PIDController::PIDController(float p, float i, float d) : 
    kP(p), kI(i), kD(d), integral(0.0f), lastError(0.0f), maxIntegral(100.0f) {}

// Initialization function implementation
void PIDController::init(float p, float i, float d) {
    kP = p; 
    kI = i; 
    kD = d;
    integral = 0.0f; // Reset state on re-initialization
    lastError = 0.0f;
}

// Calculation function implementation
float PIDController::calculate(float error, float dt) {
    if (dt <= 0.0f) return 0.0f; // Prevent division by zero

    // 1. Integral Term Update
    integral += error * dt;

    // Anti-windup mechanism: Clamp the integral term
    // This prevents large errors from causing excessively high control outputs when the system is saturated.
    integral = std::max(-maxIntegral, std::min(maxIntegral, integral));

    // 2. Derivative Term Calculation
    float derivative = (error - lastError) / dt;
    
    // Update state for next iteration
    lastError = error;

    // 3. Final Output
    // output = P * error + I * integral + D * derivative
    return (kP * error) + (kI * integral) + (kD * derivative);
}