#include <Arduino.h>
#include <cmath>
#include "FastAccelStepper.h"
#include "TrapezoidalVelocity.hpp"

#if defined(ARDUINO_ARCH_ESP32)
#include "esp32-hal-timer.h"
#include "soc/gpio_struct.h"
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

// pins
constexpr uint8_t PinDir   = 32;
constexpr uint8_t PinPulse = 33;

// step generation
constexpr uint32_t kStepsPerRevolution = 800;
constexpr uint32_t kStepPulseWidthUs = 10;  // fixed HIGH time for each step pulse

// control loop frequency
constexpr uint32_t kLoopFreqHz = 60;

// velocity profile
constexpr TrapezoidalLimits kVelocityLimits = {
   .max_duration = 60.0,
   .max_velocity = 360.0,
   .max_acceleration = 1000.0
};
static TrapezoidalVelocity velocity_profile = {};

// Mutex protecting velocity profile access
static SemaphoreHandle_t traj_mutex = nullptr;

#if defined(ARDUINO_ARCH_ESP32)
static hw_timer_t* step_timer = nullptr;
static portMUX_TYPE step_timer_mux = portMUX_INITIALIZER_UNLOCKED;

// period in microseconds; 0 means stopped
static volatile uint32_t step_period_us = 0;
static volatile bool step_pin_high = false;

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

static void IRAM_ATTR step_timer_isr() {
   portENTER_CRITICAL_ISR(&step_timer_mux);
   const uint32_t period_us = step_period_us;

   if (period_us == 0) {
      gpio_set_low(PinPulse);
      step_pin_high = false;
      timerStop(step_timer);
      portEXIT_CRITICAL_ISR(&step_timer_mux);
      return;
   }

   if (!step_pin_high) {
      // Rising edge (start pulse)
      gpio_set_high(PinPulse);
      step_pin_high = true;
      timerWrite(step_timer, 0);
      timerAlarm(step_timer, kStepPulseWidthUs, false, 0);
   } else {
      // Falling edge (end pulse)
      gpio_set_low(PinPulse);
      step_pin_high = false;
      const uint32_t low_us = (period_us > kStepPulseWidthUs) ? (period_us - kStepPulseWidthUs) : 1U;
      timerWrite(step_timer, 0);
      timerAlarm(step_timer, low_us, false, 0);
   }

   portEXIT_CRITICAL_ISR(&step_timer_mux);
}

static void set_step_rate_hz(uint32_t step_freq_hz) {
   // 0 stops pulses.
   portENTER_CRITICAL(&step_timer_mux);
   if (step_freq_hz == 0) {
      step_period_us = 0;
      gpio_set_low(PinPulse);
      step_pin_high = false;
      if (step_timer != nullptr) {
         timerStop(step_timer);
      }
      portEXIT_CRITICAL(&step_timer_mux);
      return;
   }

   // Enforce a minimum period so LOW time is at least 1us.
   const uint32_t min_period_us = kStepPulseWidthUs + 1U;
   uint32_t period_us = 1000000UL / step_freq_hz;
   if (period_us < min_period_us) {
      period_us = min_period_us;
   }

   step_period_us = period_us;

   // If timer is stopped, kick it off immediately with a low phase.
   if (step_timer != nullptr) {
      timerWrite(step_timer, 0);
      timerAlarm(step_timer, 1, false, 0);
      timerStart(step_timer);
   }

   portEXIT_CRITICAL(&step_timer_mux);
}
#endif

// 60 Hz control task (pinned to core 0 on ESP32)
static void control_task(void* /*unused*/) {
   // Use vTaskDelayUntil for consistent cadence.
   TickType_t period_ticks = static_cast<TickType_t>((configTICK_RATE_HZ + (kLoopFreqHz / 2)) / kLoopFreqHz);
   if (period_ticks < 1) {
      period_ticks = 1;
   }
   TickType_t last_wake = xTaskGetTickCount();

   for (;;) {
      vTaskDelayUntil(&last_wake, period_ticks);

      if (xSemaphoreTake(traj_mutex, 0) != pdTRUE) {
         continue;
      }
      const TrapezoidalVelocityPoint point = velocity_profile.process(micros());
      xSemaphoreGive(traj_mutex);

      // Convert requested velocity -> step frequency.
      // Assumes velocity is in deg/s.
      const double motor_steps_per_sec_f = (point.velocity * static_cast<double>(kStepsPerRevolution)) / 360.0;
      const int32_t motor_steps_per_sec = static_cast<int32_t>(std::lround(motor_steps_per_sec_f));

      // Direction is based on sign; frequency is absolute value.
      const bool forward = (motor_steps_per_sec >= 0);
      const uint32_t step_freq_hz = static_cast<uint32_t>(std::abs(motor_steps_per_sec));
      digitalWrite(PinDir, forward ? HIGH : LOW);

#if defined(ARDUINO_ARCH_ESP32)
      set_step_rate_hz(step_freq_hz);
#else
      // Fallback: uses duty-cycle square wave, not fixed-width pulses.
      if (step_freq_hz == 0) {
         noTone(PinPulse);
      } else {
         tone(PinPulse, step_freq_hz);
      }
#endif

   }
}

void setup() {

   Serial.begin(115200);
   delay(50);

   pinMode(PinDir, OUTPUT);
   digitalWrite(PinDir, LOW);

   pinMode(PinPulse, OUTPUT);
   digitalWrite(PinPulse, LOW);

#if defined(ARDUINO_ARCH_ESP32)
   // Hardware timer at 1MHz: alarm values are in microseconds.
   step_timer = timerBegin(1000000);
   if (step_timer != nullptr) {
      timerAttachInterrupt(step_timer, &step_timer_isr);
      timerWrite(step_timer, 0);
      timerAlarm(step_timer, 1, false, 0);
      timerStop(step_timer);
   } else {
      Serial.println("Failed to init step_timer");
   }
#endif

   velocity_profile.configure_limits(kVelocityLimits);

   traj_mutex = xSemaphoreCreateMutex();
   if (traj_mutex == nullptr) {
      Serial.println("Failed to create traj_mutex");
   }

   // Start the control task.
#if defined(ARDUINO_ARCH_ESP32)
   {
      const BaseType_t rc = xTaskCreatePinnedToCore(
      control_task,
      "ctrl60hz",
      4096,
      nullptr,
      2,
      nullptr,
      0
      );
      if (rc != pdPASS) {
         Serial.println("Failed to create ctrl60hz task");
      }
   }
#else
   {
      const BaseType_t rc = xTaskCreate(
      control_task,
      "ctrl60hz",
      4096,
      nullptr,
      2,
      nullptr
      );
      if (rc != pdPASS) {
         Serial.println("Failed to create ctrl60hz task");
      }
   }
#endif

   velocity_profile.reset(micros(), 0.0);
}

void loop() {
   delay(1000);
   if (xSemaphoreTake(traj_mutex, portMAX_DELAY) == pdTRUE) {
      velocity_profile.reset(micros(), 0.0);
      xSemaphoreGive(traj_mutex);
   }
   delay(1000);
   if (xSemaphoreTake(traj_mutex, portMAX_DELAY) == pdTRUE) {
      velocity_profile.set_target(micros(), 360.0, 500.0);
      xSemaphoreGive(traj_mutex);
   }
   delay(1000);
   if (xSemaphoreTake(traj_mutex, portMAX_DELAY) == pdTRUE) {
      velocity_profile.set_target(micros(), 0.0, 250.0);
      xSemaphoreGive(traj_mutex);
   }
   delay(600);
   if (xSemaphoreTake(traj_mutex, portMAX_DELAY) == pdTRUE) {
      velocity_profile.set_target(micros(), 0.0);
      xSemaphoreGive(traj_mutex);
   }
}
