#include <Arduino.h>
#include <cmath>

#include "TrapezoidalVelocity.hpp"
#include "VelocityServoStepper.hpp"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

// step generation
static VelocityServoStepper stepper = VelocityServoStepper(VelocityServoStepperConfig{
   .pin_dir = 32,
   .pin_pulse = 33,
   .steps_per_revolution = 800,
});

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

      // Set velocity in deg/s.
      stepper.set_target_velocity(point.velocity);

   }
}

void setup() {

   Serial.begin(115200);
   delay(50);

   stepper.begin();

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
