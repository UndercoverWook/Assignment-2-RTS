//Application: 02 - Rev2
//Release Type: Baseline Preemption
//Class: Real Time Systems - SP 2026
//Author: Xander Morris Levine
//AI Use: Used for code drafting, equation verification, README wording & formatting for GitHub

#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "driver/gpio.h"
#include "driver/adc.h"   // TODO1
// (math.h already included above) // TODO1

#define LED_PIN GPIO_NUM_2  // Using GPIO2 for the LED

// TODO2: LDR_PIN = GPIO32
#define LDR_PIN GPIO_NUM_32

// TODO3: GPIO32 maps to ADC1_CHANNEL_4
#define LDR_ADC_CHANNEL ADC1_CHANNEL_4

// TODO99: global defines for moving average + threshold
#define AVG_WINDOW 10

// Tune this in Wokwi by clicking the photoresistor and changing Lux.
// For Hardware Security: alert when lux is HIGH (case opened / bright light).
#define SENSOR_THRESHOLD_LUX 480

// LDR constants (Wokwi default attributes)
#define GAMMA 0.7f
#define RL10  50.0f

// Divider constants
#define VSOURCE 3.3f
#define RFIXED_OHMS 10000.0f

// Priorities (TODO8/TODO12)
#define PRIORITY_LED    1
#define PRIORITY_PRINT  2
#define PRIORITY_SENSOR 3

// -------------------- LED TASK (TODO9) --------------------
void led_task(void *pvParameters) {
    while (1) {
        gpio_set_level(LED_PIN, 1);              // ON
        vTaskDelay(pdMS_TO_TICKS(500));          // 500 ms
        gpio_set_level(LED_PIN, 0);              // OFF
        vTaskDelay(pdMS_TO_TICKS(500));          // 500 ms
    }
    vTaskDelete(NULL);
}

// -------------------- PRINT TASK (TODO10) --------------------
void print_status_task(void *pvParameters) {
    uint32_t count = 0;
    TickType_t prevTick = xTaskGetTickCount();

    while (1) {
        TickType_t nowTick = xTaskGetTickCount();
        uint32_t nowMs  = (uint32_t)pdTICKS_TO_MS(nowTick);
        uint32_t prevMs = (uint32_t)pdTICKS_TO_MS(prevTick);

        printf("[SECURITY] Heartbeat #%lu @ %lu ms (dt=%lu ms)\n",
               (unsigned long)count++,
               (unsigned long)nowMs,
               (unsigned long)(nowMs - prevMs));

        prevTick = nowTick;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    vTaskDelete(NULL);
}

// -------------------- SENSOR TASK (TODO11) --------------------
static float raw_to_voltage(int raw) {
    // ESP32 ADC raw is 0..4095 for 12-bit
    return ((float)raw * VSOURCE) / 4095.0f;  // Vout = Dout * Vmax / Dmax :contentReference[oaicite:0]{index=0}
}

static float voltage_to_rldr(float vout) {
    // Divider with RFIXED on top (to VSOURCE) and RLDR to ground, measuring at the midpoint:
    // Vout = VS * (RLDR / (RFIXED + RLDR))
    // => RLDR = RFIXED * Vout / (VS - Vout)
    float denom = (VSOURCE - vout);
    if (denom < 0.001f) denom = 0.001f; // avoid divide-by-near-zero
    return (RFIXED_OHMS * vout) / denom;
}

static float rldr_to_lux(float rldr) {
    // Wokwi’s reference conversion uses:
    // lux = pow(RL10*1e3 * pow(10, GAMMA) / R, 1/GAMMA) :contentReference[oaicite:1]{index=1}
    if (rldr < 1.0f) rldr = 1.0f;
    float numerator = (RL10 * 1000.0f) * powf(10.0f, GAMMA);
    return powf(numerator / rldr, (1.0f / GAMMA));
}

void sensor_task(void *pvParameters) {
    // ADC already configured in app_main (TODO110 handled there)

    float luxreadings[AVG_WINDOW] = {0};
    int idx = 0;
    float sum = 0;

    // Prefill readings to avoid startup spike
    for (int i = 0; i < AVG_WINDOW; i++) {
        int raw = adc1_get_raw(LDR_ADC_CHANNEL);
        float Vmeasure = raw_to_voltage(raw);          // TODO11b
        float Rmeasure = voltage_to_rldr(Vmeasure);    // TODO11c
        float lux = rldr_to_lux(Rmeasure);             // TODO11d

        luxreadings[i] = lux;
        sum += lux;
    }

    const TickType_t periodTicks = pdMS_TO_TICKS(500);
    TickType_t lastWakeTime = xTaskGetTickCount();

    TickType_t prevTick = lastWakeTime;

    while (1) {
        int raw = adc1_get_raw(LDR_ADC_CHANNEL);

        // TODO11e-g equations
        float Vmeasure = raw_to_voltage(raw);
        float Rmeasure = voltage_to_rldr(Vmeasure);
        float lux = rldr_to_lux(Rmeasure);

        // Moving average update
        sum -= luxreadings[idx];
        luxreadings[idx] = lux;
        sum += lux;
        idx = (idx + 1) % AVG_WINDOW;

        float avgLux = sum / (float)AVG_WINDOW;

        // TODO11h-i threshold check (Hardware Security: HIGH lux => possible tamper)
        if (avgLux > (float)SENSOR_THRESHOLD_LUX) {
            printf("**ALERT** [SECURITY TAMPER] AvgLux=%.1f > %d | raw=%d V=%.2f R=%.0fΩ\n",
                   avgLux, SENSOR_THRESHOLD_LUX, raw, Vmeasure, Rmeasure);
        } else {
            // optional debug line (leave on while tuning, then comment out)
            printf("[sensor] AvgLux=%.1f (thr=%d)\n", avgLux, SENSOR_THRESHOLD_LUX);
        }

        // TODO11j: print period timing info
        TickType_t nowTick = xTaskGetTickCount();
        printf("[sensor timing] now=%lu ms dt=%lu ms (target=500ms)\n",
               (unsigned long)pdTICKS_TO_MS(nowTick),
               (unsigned long)(pdTICKS_TO_MS(nowTick) - pdTICKS_TO_MS(prevTick)));
        prevTick = nowTick;

        // TODO11k: use vTaskDelayUntil for stable period
          /* BONUS STARVATION EXPERIMENT (leave commented out for submission)
        Uncomment the two lines below to cause starvation:
        - The sensor task will NEVER block
        - Because it is highest priority, it will starve LED + STATUS tasks on a single core
          */
          // continue;
          // vTaskDelay(pdMS_TO_TICKS(0));
        vTaskDelayUntil(&lastWakeTime, periodTicks);
    }

    vTaskDelete(NULL);
}

// -------------------- APP MAIN --------------------
void app_main() {
    // LED output
    gpio_reset_pin(LED_PIN);
    gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);

    // TODO4: LDR input pin
    gpio_reset_pin(LDR_PIN);
    gpio_set_direction(LDR_PIN, GPIO_MODE_INPUT);

    // TODO5: ADC width 12-bit
    adc1_config_width(ADC_WIDTH_BIT_12);

    // TODO6: ADC attenuation 11 dB on channel
    adc1_config_channel_atten(LDR_ADC_CHANNEL, ADC_ATTEN_DB_11);

    // TODO7: pin tasks to core 1 using xTaskCreatePinnedToCore
    xTaskCreatePinnedToCore(led_task, "LED", 2048, NULL, PRIORITY_LED, NULL, 1);
    xTaskCreatePinnedToCore(print_status_task, "STATUS", 2048, NULL, PRIORITY_PRINT, NULL, 1);

    // TODO12: add sensor task (bigger stack for math/prints)
    xTaskCreatePinnedToCore(sensor_task, "SENSOR", 4096, NULL, PRIORITY_SENSOR, NULL, 1);

    // app_main can return
}
