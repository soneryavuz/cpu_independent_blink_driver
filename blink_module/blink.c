#include <stdlib.h>
#include <stdint.h>
#include "blink.h"

#define TICK_DIFF(tick, interval)   (p_blink_fp_api->fp_tick_elapsed(tick, p_blink_fp_api->fp_get_current_tick()) >= interval)
#define BLINK_BUFFER_COUNT          (3)
#define BLINK_ENABLE_OFFSET_MASK    (0x01)
#define BLINK_ON_OFF_STATE_MASK     (0x02)

typedef enum
{
    BLINK_INITIALIZE_FAIL = -1,
    BLINK_INITIALIZE_SUCCESS = 0,
} blink_initialize_status_t;

enum
{
    BLINK_OFF,
    BLINK_ON
};

typedef struct 
{
    uint32_t on_time;
    uint32_t off_time;
    uint32_t offset;
    uint32_t time;
    uint32_t count;
    uint32_t cycle;
    uint8_t states;
    void * param;
} blink_t;

static blink_fp_api_t * p_blink_fp_api = NULL;
static uint32_t num_of_ticks_in_1ms = 0;
static blink_t blinks[BLINK_BUFFER_COUNT] = {{0}};
static blink_initialize_status_t initialize_status = BLINK_INITIALIZE_FAIL;


static int find_recorded_id(void * const param)
{
    int i = 0;
    int empty_index = -1;

    while (i < BLINK_BUFFER_COUNT) 
    {
        if (blinks[i].param == param) 
        {
            empty_index = i;
            break;
        }
        if (empty_index < 0 && blinks[i].param == NULL) 
        {
            empty_index = i;
        }
        i++;
    }
    return empty_index;
}

static void set_blink_state(blink_t * const p_param, const uint8_t onoff)
{
    int inx = find_recorded_id(p_param);
    if (inx >= 0)
    {
        if(onoff)
        {
            blinks[inx].states |= BLINK_ON_OFF_STATE_MASK;
        }
        else
        {
            blinks[inx].states &= ~BLINK_ON_OFF_STATE_MASK;
        }
        if(NULL != p_blink_fp_api->fp_blink_output_func)
        {
            p_blink_fp_api->fp_blink_output_func(blinks[inx].param, onoff);
        }
        blinks[inx].count++;    
    }
}


int blink_initialize(blink_fp_api_t * p_api, uint32_t count_of_ticks_in_1ms)
{
    if ((NULL != p_api)
        && (NULL != p_api->fp_blink_output_func)
        && (NULL != p_api->fp_get_current_tick)
        && (NULL != p_api->fp_tick_elapsed)
        && (0 != count_of_ticks_in_1ms))
    {
        p_blink_fp_api = p_api;
        num_of_ticks_in_1ms = count_of_ticks_in_1ms;
        initialize_status = BLINK_INITIALIZE_SUCCESS;
    }
    return initialize_status;
}

void blink_set(void * const param, const uint32_t on_time, const uint32_t off_time, const uint32_t cycle, const uint32_t offset)
{
    if (BLINK_INITIALIZE_SUCCESS == initialize_status)
    {
        int inx = find_recorded_id(param);
        if (inx >= 0)
        {
            blinks[inx].on_time = on_time;
            blinks[inx].off_time = off_time;
            blinks[inx].offset = offset;
            blinks[inx].cycle = cycle;
            blinks[inx].count = 0;
            blinks[inx].time = p_blink_fp_api->fp_get_current_tick();
            blinks[inx].param = param;
            if(0 == offset)
            {
                blinks[inx].states |= BLINK_ENABLE_OFFSET_MASK;
                set_blink_state(blinks[inx].param, BLINK_ON);
            }
        }
    }
}

void blink_process(void)
{
    if (BLINK_INITIALIZE_SUCCESS == initialize_status)
    {
        uint8_t ch = 0;
        blink_t * p_blink = NULL;
        
        while (ch < (sizeof(blinks) / sizeof(blink_t)))
        {
            p_blink = (blink_t *)&blinks[ch];
            if(NULL != p_blink->param)
            {
                if (0 == (p_blink->states & BLINK_ENABLE_OFFSET_MASK))
                {
                    if (TICK_DIFF(p_blink->time, p_blink->offset * num_of_ticks_in_1ms))
                    {
                        p_blink->states |= BLINK_ENABLE_OFFSET_MASK;
                        p_blink->time = p_blink_fp_api->fp_get_current_tick();
                        set_blink_state(p_blink->param, BLINK_ON);
                    }
                }

                if (p_blink->states & BLINK_ENABLE_OFFSET_MASK)
                {
                    if (p_blink->states & BLINK_ON_OFF_STATE_MASK)
                    {
                        if (TICK_DIFF(p_blink->time, p_blink->on_time * num_of_ticks_in_1ms))
                        {
                            p_blink->time = p_blink_fp_api->fp_get_current_tick();
                            if((0 != p_blink->cycle) && (p_blink->cycle >= p_blink->count))
                            {
                                set_blink_state(p_blink->param, (0 == p_blink->off_time) ? BLINK_ON : BLINK_OFF);
                            }
                            else
                            {
                                p_blink->states &= ~BLINK_ENABLE_OFFSET_MASK;
                                set_blink_state(p_blink->param, BLINK_OFF);
                                p_blink->param = NULL;
                            }
                        }
                    }
                    else
                    {
                        if (TICK_DIFF(p_blink->time, p_blink->off_time * num_of_ticks_in_1ms))
                        {
                            p_blink->time = p_blink_fp_api->fp_get_current_tick();
                            if((0 != p_blink->cycle) && (p_blink->cycle >= p_blink->count))
                            {
                                set_blink_state(p_blink->param, BLINK_ON);
                            }
                            else
                            {
                                p_blink->states &= ~BLINK_ENABLE_OFFSET_MASK;
                                set_blink_state(p_blink->param, BLINK_OFF);
                                p_blink->param = NULL;
                            }
                        }
                    }
                }
            }
            ch++;  
        }        
    }
}

void blink_remove(void * const param)
{
    if (BLINK_INITIALIZE_SUCCESS == initialize_status)
    {
        int inx = find_recorded_id(param);
        if (inx >= 0)
        {
            set_blink_state(param, BLINK_OFF);
            blinks[inx].states &= ~BLINK_ENABLE_OFFSET_MASK;
            blinks[inx].param = NULL;
        }
    }
}

int blink_get_state(void * const param)
{
    int ret = -1;
    if (BLINK_INITIALIZE_SUCCESS == initialize_status)
    {
        int inx = find_recorded_id(param);
        if (inx >= 0)
        {
            ret = (blinks[inx].states & BLINK_ENABLE_OFFSET_MASK);
        }
    }
    return ret;
}

void blink_force_update(void * const param, const uint8_t onoff)
{
    if (BLINK_INITIALIZE_SUCCESS == initialize_status)
    {
        int inx = find_recorded_id(param);
        if (inx >= 0)
        {
            set_blink_state(param, onoff);
        }
    }
}