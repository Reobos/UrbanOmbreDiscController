#pragma once

#include <cstdint>

struct VelocityServoStepperConfig {
    uint8_t pin_dir = 0;
    uint8_t pin_pulse = 0;
    uint32_t steps_per_revolution = 0;
};

class VelocityServoStepper {

    static constexpr uint32_t kStepPulseWidthUs = 10;  // fixed HIGH time for each step pulse

public:
    VelocityServoStepper() = default;
    VelocityServoStepper(const VelocityServoStepperConfig& config);

    // Call from Arduino setup() after Serial.begin()/pinMode is available.
    // Initializes GPIO and the underlying pulse generator.
    void begin();

    // Stops pulses immediately and drives pulse pin low.
    void stop();

    // Velocity command in deg/s (positive/negative sets direction).
    void set_target_velocity(const double velocity_deg_per_sec);

    // Direct step-rate command in steps/s (positive/negative sets direction).
    void set_target_steps_per_sec(const int32_t steps_per_sec);

private:
#if defined(ARDUINO_ARCH_ESP32)
    static void step_timer_isr();
    void set_step_rate_hz(uint32_t step_freq_hz);
#endif

    uint8_t m_pin_dir = 0;
    uint8_t m_pin_pulse = 0;
    uint32_t m_steps_per_revolution = 0;
};
