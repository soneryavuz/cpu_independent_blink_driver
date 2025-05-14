<h1 align="center">Blink Driver API Documentation</h1>

*This blink driver depends only on the standard C library and is completely processor-agnostic.*
A lightweight, parameterized blink manager for embedded systems. This driver allows you to schedule and manage multiple blink events on any GPIO-like resource, with configurable on-time, off-time, offset, and cycle count.

## Features

* Supports up to `BLINK_BUFFER_COUNT` simultaneous blink instances (default: 3).
* Configurable `on_time_ms`, `off_time_ms`, and `offset` durations in milliseconds.
* Cycle count: automatically stop after a given number of on/off toggles.
* Zero `off_time_ms` special case: continuous ON for the specified cycle count.
* Non-blocking: driven by periodic calls to `blink_process()`.
* User-provided tick and output callbacks via `blink_initialize()`.

---

## API Reference

### 1. `int blink_initialize(blink_fp_api_t *p_api, uint32_t ticks_per_ms)`

Initialize the blink driver with your platform callbacks.

* **Parameters**:

  * `p_api`: Pointer to a `blink_fp_api_t` struct containing:

    * `fp_blink_output_func(void *param, uint8_t onoff)`: Output callback (e.g., toggles a GPIO).
    * `fp_get_current_tick()`: Returns current tick count (monotonic timer).
    * `fp_tick_elapsed(start, now)`: Returns elapsed ticks between `start` and `now`.
  * `ticks_per_ms`: Number of hardware ticks in one millisecond.

* **Returns**:

  * `BLINK_INITIALIZE_SUCCESS` (0) on success, or `BLINK_INITIALIZE_FAIL` (-1) on error (invalid args).

---

### 2. `void blink_set(void *param, uint32_t on_time_ms, uint32_t off_time_ms, uint32_t cycle, uint32_t offset)`

Schedule a blink instance on the resource identified by `param`.

* **Parameters**:

  * `param`: User identifier passed to the output callback (e.g., pointer to a GPIO pin).
  * `on_time_ms`: Duration (ms) LED stays ON.
  * `off_time_ms`: Duration (ms) LED stays OFF; set `0` to remain continuously ON for `cycle` events.
  * `cycle`: Number of ON/OFF toggles; when reached, the blink instance stops and frees its slot.
  * `offset`: Initial delay (ms) before the first ON event.

**Note:** If no free slot is available, the request is ignored.

---

### 3. `void blink_process(void)`

Must be called periodically (e.g., from a timer interrupt or main loop). Handles all timing, state changes, and invokes the output callback as needed.

---

### 4. `void blink_remove(void *param)`

Immediately stop and remove any active blink instance associated with `param`.

* **Parameters**:

  * `param`: Identifier of the blink instance to remove.

---

### 5. `int blink_get_state(void *param)`

Query whether the blink identified by `param` is currently active (offset enabled).

* **Returns**:

  * `>=0`: Active state mask (non-zero if active).
  * `-1`: Not initialized or not found.

---

### 6. `void blink_force_update(void *param, uint8_t onoff)`

Force an immediate ON or OFF event for the given `param`, without affecting cycle count.

* **Parameters**:

  * `param`: Identifier of the blink instance.
  * `onoff`: `BLINK_ON` (1) or `BLINK_OFF` (0).

---

## Configuration Macros

```c
#define BLINK_BUFFER_COUNT       3   // Max concurrent blink slots
#define BLINK_ENABLE_OFFSET_MASK 0x01
#define BLINK_ON_OFF_STATE_MASK  0x02
#define TICK_DIFF(t, iv)         (p_blink_fp_api->fp_tick_elapsed(t, p_blink_fp_api->fp_get_current_tick()) >= (iv))
```

---

## Example Application

> The following `app_main` example shows how to integrate and use the Blink Driver in an ESP-IDF project. It configures a GPTimer for a millisecond tick source, initializes two LED GPIOs, registers the blink API callbacks, and schedules two blink patterns:

```c
#include <stdio.h>
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "../blink_module/blink.h"

#define LED1_GPIO   18
#define LED2_GPIO   19
#define SYSTEM_FREQUENCY    (40000000U)
#define MSEC_TO_TICK(ms)    ((ms) * (SYSTEM_FREQUENCY / 1000))

static gptimer_handle_t gptimer = NULL;
static blink_fp_api_t blink_fp_api;
static uint64_t current_tick = 0;

static uint32_t get_current_tick(void)
{
    gptimer_get_raw_count(gptimer, &current_tick);
    return (uint32_t)current_tick;
}

static uint32_t tick_elapsed(uint32_t start, uint32_t end)
{
    if (end >= start) {
        return end - start;
    }
    return (UINT32_MAX - start) + end;
}

static void set_blink_callback(void * const param, uint8_t onoff)
{
    uint8_t pin = *(uint8_t *)param;
    gpio_set_level(pin, onoff);
}

static void system_initialize(void)
{
    // Timer initialization
    gptimer_config_t timer_config = {
        .clk_src      = GPTIMER_CLK_SRC_DEFAULT,
        .direction    = GPTIMER_COUNT_UP,
        .resolution_hz= SYSTEM_FREQUENCY,
    };
    esp_err_t err = gptimer_new_timer(&timer_config, &gptimer);
    ESP_ERROR_CHECK(err);
    ESP_ERROR_CHECK(gptimer_set_raw_count(gptimer, 0));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_ERROR_CHECK(gptimer_start(gptimer));

    // GPIO configuration
    gpio_config_t io_conf = {
        .intr_type    = GPIO_INTR_DISABLE,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pin_bit_mask = (1ULL << LED1_GPIO) | (1ULL << LED2_GPIO),
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // Blink API registration
    blink_fp_api.fp_blink_output_func = set_blink_callback;
    blink_fp_api.fp_get_current_tick   = get_current_tick;
    blink_fp_api.fp_tick_elapsed       = tick_elapsed;
    ESP_ERROR_CHECK(blink_initialize(&blink_fp_api, MSEC_TO_TICK(1)));
}

void app_main(void)
{
    system_initialize();

    static uint8_t blink_io[2] = { LED1_GPIO, LED2_GPIO };

    // First LED: 100 ms ON, 100 ms OFF, no offset, repeated 200 times
    blink_set((void *)&blink_io[0], 100, 100, 200, 0);
    // Second LED: continuous ON for 200 cycles (off_time=0), with a 1000 ms initial delay
    blink_set((void *)&blink_io[1], 100, 0, 200, 1000);

    while (1) {
        blink_process();
    }
}
```

---

## Header File Explanation

The `blink.h` header defines the public interface for the Blink Driver:

```c
#ifndef BLINK_H
#define BLINK_H

typedef struct
{
    void     (*fp_blink_output_func)(void * const param, uint8_t onoff);
    uint32_t (*fp_get_current_tick)(void);
    uint32_t (*fp_tick_elapsed)(uint32_t start, uint32_t end);
} blink_fp_api_t;

int  blink_initialize(blink_fp_api_t *p_api, uint32_t ticks_per_ms);
void blink_set(void * const param,
               uint32_t on_time_ms,
               uint32_t off_time_ms,
               uint32_t cycle,
               uint32_t offset);
void blink_process(void);
void blink_remove(void * const param);
int  blink_get_state(void * const param);
void blink_force_update(void * const param, uint8_t onoff);

#endif
```

**Callback Struct Fields:**

* `fp_blink_output_func`: Application-provided callback to set ON/OFF state (`onoff`) for the resource.
* `fp_get_current_tick`: Callback to get the current tick count from the timer.
* `fp_tick_elapsed`: Callback to compute the elapsed ticks between two tick values (`start`, `end`).

1. **`blink_fp_api_t`**: Structure of function pointers to integrate with your timer and output mechanism.
2. **`blink_initialize`**: Must be called once before any blinking; registers callbacks and tick frequency.
3. **`blink_set`**: Schedules a blink instance on the resource identified by `param`.
4. **`blink_process`**: Drives the state machine; call periodically to handle timing and transitions.
5. **`blink_remove`**: Stops and removes an active blink instance immediately.
6. **`blink_get_state`**: Returns whether a blink instance is currently active.
7. **`blink_force_update`**: Forces immediate ON/OFF output without altering the cycle count.

---