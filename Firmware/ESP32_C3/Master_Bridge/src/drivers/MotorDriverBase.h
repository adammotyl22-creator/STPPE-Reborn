#pragma once
#include <Arduino.h>

/**
 * @brief Abstract Base Class for all motor drivers on the platform.
 * 
 * Enforces a common, non-blocking API regardless of underlying hardware (TMC2209, Servo, etc.).
 * All concrete implementations must inherit from this class and implement these methods.
 */
class MotorDriverBase {
public:
    MotorDriverBase(const char* name) : name_(name), currentPosition_(0.0f) {}
    virtual ~MotorDriverBase() = default;

    /**
     * @brief Initializes the motor driver hardware (e.g., setting pins, communicating with HW).
     */
    virtual bool initializeHardware() = 0;

    /**
     * @brief Sets a target angular position (degrees) for the next move segment.
     * Must be non-blocking and update internal state machine/target values.
     * @param degrees The target angle in degrees relative to the motor's home position.
     */
    virtual void setTargetPosition(float degrees) = 0;

    /**
     * @brief Updates the motor based on elapsed time and speed constraints.
     * Must be called repeatedly within the main loop(). Returns true if movement occurred, false otherwise.
     * This function encapsulates all blocking/non-blocking motion control logic (PID/Accel).
     * 
     * @param deltaTimeMs The time elapsed since the last call, in milliseconds.
     * @return bool True if motor was actively moving, false if stationary or error state.
     */
    virtual bool update(unsigned long deltaTimeMs) = 0;

    /**
     * @brief Gets the current physical position of the joint (degrees).
     * Must be thread-safe and reflect the last successful measurement/interpolation.
     */
    float getCurrentPosition() const { return currentPosition_; }

    const char* getName() const { return name_; }

protected:
    // Protected members force subclasses to manage state variables like this
    std::string name_;
    float currentPosition_; // In degrees
};