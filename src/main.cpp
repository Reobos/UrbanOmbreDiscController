#include <Arduino.h>
#include <cmath>

#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>

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

namespace {

// Simple ESP-NOW packet format for velocity commands.
// Sender must send exactly sizeof(EspNowVelocityCmd) bytes.
struct EspNowVelocityCmd {
   uint32_t magic;
   uint16_t version;
   uint16_t _reserved;
   float velocity_deg_s;
   float acceleration_deg_s2;
   uint32_t seq;
};

constexpr uint32_t kEspNowMagic = 0x56454C43u; // 'VELC'
constexpr uint16_t kEspNowVersion = 1;

// If no command received within this window, we command 0 velocity.
constexpr uint32_t kCmdTimeoutMs = 1000;

portMUX_TYPE g_cmd_mux = portMUX_INITIALIZER_UNLOCKED;
volatile EspNowVelocityCmd g_last_cmd = {
   .magic = kEspNowMagic,
   .version = kEspNowVersion,
   ._reserved = 0,
   .velocity_deg_s = 0.0f,
   .acceleration_deg_s2 = 500.0f,
   .seq = 0,
};
volatile uint32_t g_last_cmd_rx_ms = 0;

static void on_espnow_recv(const esp_now_recv_info_t* /*recv_info*/, const uint8_t* data, int len) {
   if (data == nullptr || len != static_cast<int>(sizeof(EspNowVelocityCmd))) {
      return;
   }

   EspNowVelocityCmd cmd{};
   memcpy(&cmd, data, sizeof(cmd));
   if (cmd.magic != kEspNowMagic || cmd.version != kEspNowVersion) {
      return;
   }

   portENTER_CRITICAL_ISR(&g_cmd_mux);
   memcpy((void*)&g_last_cmd, &cmd, sizeof(cmd));
   g_last_cmd_rx_ms = millis();
   portEXIT_CRITICAL_ISR(&g_cmd_mux);
}

static bool espnow_begin() {
   WiFi.mode(WIFI_STA);
   WiFi.disconnect(true, true);

   // Optional: if you want to lock channel, uncomment and set a channel that matches the sender.
   // constexpr uint8_t kChannel = 1;
   // esp_wifi_set_promiscuous(true);
   // esp_wifi_set_channel(kChannel, WIFI_SECOND_CHAN_NONE);
   // esp_wifi_set_promiscuous(false);

   if (esp_now_init() != ESP_OK) {
      return false;
   }
   if (esp_now_register_recv_cb(on_espnow_recv) != ESP_OK) {
      return false;
   }

   return true;
}

static bool get_latest_cmd(float* velocity_deg_s, float* acceleration_deg_s2, uint32_t* age_ms) {
   if (velocity_deg_s == nullptr || acceleration_deg_s2 == nullptr || age_ms == nullptr) {
      return false;
   }

   EspNowVelocityCmd cmd{};
   uint32_t rx_ms = 0;
   portENTER_CRITICAL(&g_cmd_mux);
   memcpy(&cmd, (const void*)&g_last_cmd, sizeof(cmd));
   rx_ms = g_last_cmd_rx_ms;
   portEXIT_CRITICAL(&g_cmd_mux);

   const uint32_t now_ms = millis();
   *age_ms = (rx_ms == 0) ? 0xFFFFFFFFu : (now_ms - rx_ms);
   *velocity_deg_s = cmd.velocity_deg_s;
   *acceleration_deg_s2 = cmd.acceleration_deg_s2;
   return true;
}

} // namespace

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

      if (traj_mutex == nullptr) {
         continue;
      }

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

   {
      uint8_t mac[6] = {0};
      WiFi.macAddress(mac);
      Serial.print("ESP32 STA MAC: ");
      Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

      if (!espnow_begin()) {
         Serial.println("ESP-NOW init failed (velocity cmds disabled)");
      } else {
         Serial.println("ESP-NOW receiver ready");
      }
   }

   // Start the control task.
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

   velocity_profile.reset(micros(), 0.0);
}

void loop() {
   delay(50);

   float velocity_cmd_deg_s = 0.0f;
   float accel_cmd_deg_s2 = 500.0f;
   uint32_t age_ms = 0xFFFFFFFFu;

   (void)get_latest_cmd(&velocity_cmd_deg_s, &accel_cmd_deg_s2, &age_ms);
   if (age_ms > kCmdTimeoutMs) {
      velocity_cmd_deg_s = 0.0f;
   }

   if (traj_mutex != nullptr && xSemaphoreTake(traj_mutex, portMAX_DELAY) == pdTRUE) {
      velocity_profile.set_target(micros(), static_cast<double>(velocity_cmd_deg_s), static_cast<double>(accel_cmd_deg_s2));
      xSemaphoreGive(traj_mutex);
   }

   Serial.print("Speed cmd [deg/s], age [ms]: ");
   Serial.print(velocity_cmd_deg_s);
   Serial.print(", ");
   Serial.println(age_ms);
}
