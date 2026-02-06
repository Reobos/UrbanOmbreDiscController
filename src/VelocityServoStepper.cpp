#include "VelocityServoStepper.hpp"

#include <Arduino.h>
#include <cmath>

#if defined(ARDUINO_ARCH_ESP32)
#include "esp32-hal-timer.h"
#include "soc/gpio_struct.h"
#include "freertos/FreeRTOS.h"
#include "freertos/portmacro.h"
#endif

VelocityServoStepper::VelocityServoStepper(const VelocityServoStepperConfig& config)
    : m_pin_dir(config.pin_dir),
      m_pin_pulse(config.pin_pulse),
      m_steps_per_revolution(config.steps_per_revolution)
{
}

#if defined(ARDUINO_ARCH_ESP32)
namespace {
// This implementation supports one active instance.
static VelocityServoStepper* g_instance = nullptr;

static hw_timer_t* g_step_timer = nullptr;
static portMUX_TYPE g_timer_mux = portMUX_INITIALIZER_UNLOCKED;

// period in microseconds; 0 means stopped
static volatile uint32_t g_step_period_us = 0;
static volatile bool g_step_pin_high = false;

static inline void gpio_set_high(uint8_t pin) {
    if (pin < 32) {
        GPIO.out_w1ts = (1UL << pin);
    } else {
        GPIO.out1_w1ts.val = (1UL << (pin - 32));
    }
}

static inline void gpio_set_low(uint8_t pin) {
    if (pin < 32) {
        GPIO.out_w1tc = (1UL << pin);
    } else {
        GPIO.out1_w1tc.val = (1UL << (pin - 32));
    }
}
}  // namespace
#endif

#if defined(ARDUINO_ARCH_ESP32)
void IRAM_ATTR VelocityServoStepper::step_timer_isr() {
    portENTER_CRITICAL_ISR(&g_timer_mux);

    const uint32_t period_us = g_step_period_us;

    if (period_us == 0 || g_step_timer == nullptr || g_instance == nullptr) {
        if (g_instance != nullptr) {
            gpio_set_low(g_instance->m_pin_pulse);
        }
        g_step_pin_high = false;
        if (g_step_timer != nullptr) {
            timerStop(g_step_timer);
        }
        portEXIT_CRITICAL_ISR(&g_timer_mux);
        return;
    }

    const uint8_t pulse_pin = g_instance->m_pin_pulse;

    if (!g_step_pin_high) {
        // Rising edge (start pulse)
        gpio_set_high(pulse_pin);
        g_step_pin_high = true;
        timerWrite(g_step_timer, 0);
        timerAlarm(g_step_timer, kStepPulseWidthUs, false, 0);
    } else {
        // Falling edge (end pulse)
        gpio_set_low(pulse_pin);
        g_step_pin_high = false;
        const uint32_t low_us = (period_us > kStepPulseWidthUs) ? (period_us - kStepPulseWidthUs) : 1U;
        timerWrite(g_step_timer, 0);
        timerAlarm(g_step_timer, low_us, false, 0);
    }

    portEXIT_CRITICAL_ISR(&g_timer_mux);
}

void VelocityServoStepper::set_step_rate_hz(uint32_t step_freq_hz) {
    portENTER_CRITICAL(&g_timer_mux);

    if (step_freq_hz == 0) {
        g_step_period_us = 0;
        gpio_set_low(m_pin_pulse);
        g_step_pin_high = false;
        if (g_step_timer != nullptr) {
            timerStop(g_step_timer);
        }
        portEXIT_CRITICAL(&g_timer_mux);
        return;
    }

    // Enforce a minimum period so LOW time is at least 1us.
    const uint32_t min_period_us = kStepPulseWidthUs + 1U;
    uint32_t period_us = 1000000UL / step_freq_hz;
    if (period_us < min_period_us) {
        period_us = min_period_us;
    }

    g_step_period_us = period_us;

    // If timer is stopped, kick it off immediately with a low phase.
    if (g_step_timer != nullptr) {
        timerWrite(g_step_timer, 0);
        timerAlarm(g_step_timer, 1, false, 0);
        timerStart(g_step_timer);
    }

    portEXIT_CRITICAL(&g_timer_mux);
}
#endif

void VelocityServoStepper::begin() {
    pinMode(m_pin_dir, OUTPUT);
    digitalWrite(m_pin_dir, LOW);

    pinMode(m_pin_pulse, OUTPUT);
    digitalWrite(m_pin_pulse, LOW);

#if defined(ARDUINO_ARCH_ESP32)
    // This implementation supports a single active instance.
    g_instance = this;

    // Hardware timer at 1MHz: alarm values are in microseconds.
    g_step_timer = timerBegin(1000000);
    if (g_step_timer != nullptr) {
        timerAttachInterrupt(g_step_timer, &VelocityServoStepper::step_timer_isr);
        timerWrite(g_step_timer, 0);
        timerAlarm(g_step_timer, 1, false, 0);
        timerStop(g_step_timer);
    } else {
        Serial.println("Failed to init step_timer");
    }
#endif
}

void VelocityServoStepper::stop() {
#if defined(ARDUINO_ARCH_ESP32)
    if (g_instance == this) {
        set_step_rate_hz(0);
    } else {
        digitalWrite(m_pin_pulse, LOW);
    }
#else
    noTone(m_pin_pulse);
    digitalWrite(m_pin_pulse, LOW);
#endif
}

void VelocityServoStepper::set_target_velocity(const double velocity_deg_per_sec) {
    // Convert deg/s -> steps/s, preserving sign.
    const double steps_per_sec_f = (velocity_deg_per_sec * static_cast<double>(m_steps_per_revolution)) / 360.0;
    const int32_t steps_per_sec = static_cast<int32_t>(std::lround(steps_per_sec_f));
    set_target_steps_per_sec(steps_per_sec);
}

void VelocityServoStepper::set_target_steps_per_sec(const int32_t steps_per_sec) {
    const bool forward = (steps_per_sec >= 0);
    digitalWrite(m_pin_dir, forward ? HIGH : LOW);

    const uint32_t step_freq_hz = static_cast<uint32_t>(std::abs(steps_per_sec));

#if defined(ARDUINO_ARCH_ESP32)
    if (g_instance == this) {
        set_step_rate_hz(step_freq_hz);
    }
#else
    // Fallback: uses duty-cycle square wave, not fixed-width pulses.
    if (step_freq_hz == 0) {
        noTone(m_pin_pulse);
        digitalWrite(m_pin_pulse, LOW);
    } else {
        tone(m_pin_pulse, step_freq_hz);
    }
#endif
}
