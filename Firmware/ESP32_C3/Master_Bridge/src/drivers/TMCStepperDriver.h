#pragma once
#include "MotorDriverBase.h"
#include <TMCStepper.h>
// Assuming Serial4 is configured and available for communication
extern TMC2209Stepper* global_serial_port;

/**
 * @brief Concrete driver implementation for high-fidelity TMC steppers (e.g., J1, J2, J3).
 */
class TMCStepperDriver : public MotorDriverBase {
public:
    TMCStepperDriver(const char* name, TMC2209Stepper* tmc);

    bool initializeHardware() override;
    void setTargetPosition(float degrees) override;
    bool update(unsigned long deltaTimeMs) override;

private:
    TMC2209Stepper* tmc_driver_; // Pointer to the specific HW driver object
};