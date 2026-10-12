#pragma once
#include <Arduino.h>
#include <cmath> // For fabsf, constrain

/**
 * @brief A robust PID controller class suitable for embedded RTOS environments.
 * 
 * Implements standard PID logic with anti-windup clamping and deadband control.
 * All math is performed using floats/doubles to maintain required precision.
 */
class PIDController {
public:
    // Gains (can be set during initialization)
    float kP, kI, kD;

    PIDController(float p = 0.0f, float i = 0.0f, float d = 0.0f);

    /**
     * @brief Initializes the PID controller with specific gains.
     * @param p Proportional gain (kP)
     * @param i Integral gain (kI)
     * @param d Derivative gain (kD)
     */
    void init(float p, float i, float d);

    /**
     * @brief Calculates the control output for the current error.
     * 
     * This function is non-blocking and handles integral windup protection via clamping.
     * 
     * @param error The difference between the target setpoint and the measured value.
     * @param dt The time delta since the last call, in seconds (or proportional unit). Use a fixed interval for consistent results.
     * @return float The calculated control output value.
     */
    float calculate(float error, float dt);

private:
    // State variables
    float integral;
    float lastError;
    float maxIntegral; // For anti-windup clamping
};