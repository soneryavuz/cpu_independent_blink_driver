#include <stdio.h>
#include "driver/gpio.h"
#include "driver/gptimer.h"
#include "../blink_module/blink.h"

#define LED1_GPIO   (18)
#define LED2_GPIO   (19)
#define SYSTEM_FREQUENCY    (40000000U) //40mhz
#define MSEC_TO_TICK(ms)        (ms * (SYSTEM_FREQUENCY / 1000))
#define GET_CURRENT_TIMER_TICK  (gptimer_get_raw_count(gptimer, &current_tick)) 


static gptimer_handle_t gptimer = NULL;
static blink_fp_api_t blink_fp_api;
static uint64_t current_tick = 0;


/**
 * @fn get_current_tick
 * @brief Retrieve the current hardware timer tick count.
 * Queries the configured GPTimer and returns its raw count value as a 32-bit tick.
 * @return Current hardware timer tick count.
 */
static uint32_t get_current_tick(void)
{
    GET_CURRENT_TIMER_TICK;
    return (uint32_t)current_tick;
}

/**
 * @fn tick_elapsed
 * @brief Compute elapsed ticks between two timer readings.
 * Calculates the difference between a start and end tick, handling wrap-around.
 * @param start  The tick count at the beginning of the interval.
 * @param end    The tick count at the end of the interval.
 * @return Number of ticks elapsed between start and end.
 */
static uint32_t tick_elapsed(uint32_t start, uint32_t end)
{
    uint32_t diff = 0;
    if(end>start)
    {
        diff = (end - start);
    }
    else
    {
        diff = (UINT32_MAX - start ) +end ;
    }
    return diff;
}

/**
 * @fn set_blink_callback
 * @brief Drive the output GPIO based on blink state.
 * User-provided callback invoked by the blink driver to set the GPIO level.
 * @param param  Pointer to a uint8_t holding the GPIO number.
 * @param onoff  Desired output level: 1 = ON (HIGH), 0 = OFF (LOW).
 */
static void set_blink_callback(void * const param, uint8_t onoff)
{
    uint8_t pin = *(uint8_t*)param;
    gpio_set_level(pin, onoff);
}

static void system_initialize(void)
{
    // -------------general purpose timer initialization begin------------------
    gptimer_config_t timer_config =
    {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction= GPTIMER_COUNT_UP,
        .resolution_hz = SYSTEM_FREQUENCY, 
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));
    ESP_ERROR_CHECK(gptimer_set_raw_count(gptimer,0));
    ESP_ERROR_CHECK(gptimer_enable(gptimer));
    ESP_ERROR_CHECK(gptimer_start(gptimer));
    // -------------general purpose timer initialization end--------------------

    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    io_conf.pin_bit_mask = (1ULL << LED1_GPIO) | (1ULL << LED2_GPIO);
    gpio_config(&io_conf);
    blink_fp_api.fp_blink_output_func = set_blink_callback;
    blink_fp_api.fp_get_current_tick = get_current_tick;
    blink_fp_api.fp_tick_elapsed = tick_elapsed;
    blink_initialize(&blink_fp_api, MSEC_TO_TICK(1));
}

void app_main(void)
{
    system_initialize();

    static uint8_t blink_io[2] = {LED1_GPIO, LED2_GPIO};
    blink_set((void *)&blink_io[0], 100, 100, 200, 0);
    blink_set((void *)&blink_io[1], 100, 0, 200, 1000);
    while (1) 
    {
        blink_process();
    }
}
