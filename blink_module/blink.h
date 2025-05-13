#ifndef BLINK_H
#define BLINK_H

typedef struct
{
    void (* fp_blink_output_func)(void * const param, uint8_t onoff);
    uint32_t (* fp_get_current_tick)(void);
    uint32_t (* fp_tick_elapsed)(uint32_t start, uint32_t end);
} blink_fp_api_t;

extern int blink_initialize(blink_fp_api_t * p_api, uint32_t count_of_ticks_in_1ms);
extern void blink_set(void * const param, const uint32_t on_time, const uint32_t off_time, const uint32_t cycle, const uint32_t offset);
extern void blink_process(void);
extern void blink_remove(void * const param);
extern int blink_get_state(void * const param);
extern void blink_force_update(void * const param, const uint8_t onoff);

#endif