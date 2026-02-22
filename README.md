# Assignment-2-RTS

# RTS-SP26 Application 2 — Preemptive Scheduling w/ Sensor Integration (ESP32 + FreeRTOS)

## Project Summary
This project implements three FreeRTOS tasks on an ESP32 (Wokwi simulation), all pinned to **core 1** to clearly observe single-core scheduling behavior:

- **LED Task (priority 1 / Low):** blinks LED on GPIO2 at **1 Hz** using `vTaskDelay()`.
- **Status/Print Task (priority 2 / Medium):** prints a heartbeat message every **1000 ms** using `vTaskDelay()`.
- **Sensor Task (priority 3 / High):** samples a photoresistor (GPIO32 / ADC1_CH4) every **500 ms**, computes Lux + moving average, and prints an alert if a threshold is exceeded. Uses `vTaskDelayUntil()` for a stable periodic rate.

**Theme:** Hardware Security — the LDR simulates a light sensor inside a secure enclosure (unexpected light implies possible tampering).

##Wokwi Project Link: 
https://wokwi.com/projects/456677076049609729

##1 Task Timing and Jitter
**Sensor task timing is the most regular** because it uses `vTaskDelayUntil(&lastWakeTime, periodTicks)` with a fixed **500 ms period**. In my serial output, the sensor timing lines consistently show:

- `dt=500 ms (target=500ms)` at 500 ms, 1000 ms, 1500 ms, 2000 ms, etc.

Example from my run:
- `[sensor timing] now=500 ms dt=500 ms (target=500ms)`
- `[sensor timing] now=1000 ms dt=500 ms (target=500ms)`
- `[sensor timing] now=1500 ms dt=500 ms (target=500ms)`

This happens because `vTaskDelayUntil()` delays until an **absolute wake time** based on the original reference tick (`lastWakeTime`). Even if the task takes a little longer sometimes, the kernel still targets the next “scheduled” release time instead of “delay from now.”

The **console print task** is also very regular in this run, but it uses `vTaskDelay(1000 ms)` (relative delay). My log shows:
- `[SECURITY] Heartbeat #1 @ 1000 ms (dt=1000 ms)`
- `[SECURITY] Heartbeat #2 @ 2000 ms (dt=1000 ms)`
- `[SECURITY] Heartbeat #3 @ 3000 ms (dt=1000 ms)`

In general, tasks that use `vTaskDelay()` can show **small jitter** because they sleep for “X ticks from whenever they happened to run,” not from a fixed schedule. If the high-priority sensor task becomes ready right when the print or LED task is ready, the lower-priority tasks can be delayed slightly (they wait in the Ready state until the sensor task runs and blocks again). That delay causes variation in when they *actually* run and therefore can create drift over many iterations.

The **LED blink task** uses `vTaskDelay(500 ms)` between ON and OFF, so its period is “good enough” for a status indicator (phase drift is not critical). Even if there is small jitter, visually it still appears as a steady blink in Wokwi.



##2 Priority-Based Preemption
This system demonstrates **priority-based preemptive scheduling**: the FreeRTOS scheduler always selects the **highest-priority Ready task**.

My priorities were:
- Sensor Task = **3 (High)**
- Print Task = **2 (Medium)**
- LED Task = **1 (Low)**

A common preemption scenario occurs around **1000 ms**, when both the sensor task (500 ms period) and the print task (1000 ms period) become Ready at roughly the same time. In my output, around 1000 ms the sensor prints its timing line, and then the heartbeat prints at 1000 ms:

- sensor executes first (because it has higher priority)
- then print task runs after sensor blocks

Conceptually, if the print task were actively running and the sensor task unblocked (became Ready), the scheduler would switch to the sensor task **immediately** (preempt) because the sensor is higher priority. The print task would resume only after the sensor task blocks again (via `vTaskDelayUntil`). This is exactly why assigning the sensor task a higher priority ensures it meets its timing requirements.


##3 Effect of Task Execution Time
Right now all tasks do minimal work and block quickly, so CPU utilization is low and everything meets its intended periods.

If the **sensor task execution time increased significantly** (ex: a complex calculation taking ~300 ms each cycle), then the lower-priority tasks would get less CPU time and would likely show more jitter. Because the sensor task is the highest priority, whenever it becomes Ready it will run first. If it consumes a large portion of the CPU, the LED and print tasks might:
- print late (heartbeat timestamps might slip)
- blink irregularly or appear to “pause” briefly
- experience increased jitter (variable `dt`)

If sensor execution time sometimes **exceeds its 500 ms period**, then the task can’t finish before its next release time. Symptoms would include:
- sensor timing `dt` no longer matching 500 ms (it may become > 500 ms)
- effectively “missed deadlines” for sensor samples (sampling rate drops)
- lower-priority tasks may be heavily delayed or starved

This relates to real-time scheduling theory: the CPU must have enough utilization headroom for the periodic task set to meet deadlines. When a high-priority task overruns or uses too much CPU, lower-priority tasks suffer and the system can become unstable.

Design options if this happens:
- reduce sensor workload (optimize code, reduce math, lower print frequency)
- increase the sensor period (sample less often)
- use a more efficient filter (smaller moving average window)
- adjust priorities if appropriate (only if the sensor is not truly time-critical)
- split work across both ESP32 cores (move non-critical tasks to separate cores)
- use event-driven design (only compute heavy work when needed)


##4 vTaskDelay vs vTaskDelayUntil
I used `vTaskDelayUntil()` for the sensor task because it supports **true periodic scheduling** with minimal drift. The sensor should sample at a consistent rate (every 500 ms) to maintain deterministic behavior.

- `vTaskDelay(x)` delays for **x ticks from the time the task calls it**.  
  If the task runs late due to preemption or variable execution time, that lateness gets added into the next cycle → drift accumulates.
- `vTaskDelayUntil(&lastWakeTime, x)` delays until the next **absolute** scheduled time based on `lastWakeTime`.  
  This prevents drift by keeping the task aligned to the intended periodic schedule.

In my output, the sensor timing consistently reports:
- `dt=500 ms (target=500ms)`  
which is evidence that `vTaskDelayUntil()` is holding a stable sampling interval.

For the LED blink task, `vTaskDelay()` is acceptable because an LED status indicator does not require precise phase alignment. Small drift is not critical; the LED still communicates “alive/heartbeat” to a user.

---

##5 Thematic Integration Reflection (Hardware Security)
In the Hardware Security context:
- **LED Task (Low priority)** represents a “device power/heartbeat” indicator. It is helpful but not mission-critical.
- **Print Task (Medium priority)** represents periodic logging/telemetry to the console (system status/health).
- **Sensor Task (High priority)** represents a tamper-detection sensor inside a secure enclosure. If light enters the case (unexpected Lux), we want the system to detect and report it quickly.

The priority assignment reflects real-world importance:
- Tamper detection is time-critical (security event) → **highest priority**
- Status logging is useful but less urgent → medium priority
- LED heartbeat is least critical → lowest priority

In my run, the sensor task repeatedly printed:
- `**ALERT** [SECURITY TAMPER] AvgLux=499.6 > 480 ...`  
showing that the system detected a simulated tamper/light condition.

---

##6 Bonus — Starvation Experiment (commented out in final submission)
To demonstrate starvation, I modified the sensor task to **never block**, so it stayed Ready forever. Because it has the highest priority, it dominated the CPU and the LED/print tasks stopped running (or ran extremely rarely).

Example starvation code:
```c
/*
while (1) {
    int raw = adc1_get_raw(LDR_ADC_CHANNEL);
    float Vmeasure = raw_to_voltage(raw);
    float Rmeasure = voltage_to_rldr(Vmeasure);
    float lux = rldr_to_lux(Rmeasure);

    // No vTaskDelayUntil() or vTaskDelay() here -> sensor task never blocks
    // This starves lower-priority tasks (LED + STATUS) on a single core.
}
*/

