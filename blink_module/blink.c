/***************************************************
 * @file    blink.c                                *
 * @brief   CPU-independent blink driver           *
 * @author  Soner Yavuz                            *
 * @date    2025-05-14                             *
 *                                                 *
 * Description:                                    *
 * This driver provides a platform-agnostic        *
 * interface for managing multiple blink events    *
 * on any GPIO-like resource. Features include:    *
 *   - Configurable on/off durations, offsets,     *
 *     and cycle counts                            *
 *   - Support for continuous-ON mode (off_time=0) *
 *   - Non-blocking operation via periodic         *
 *     blink_process() calls                       *
 *   - Flexible timing through user-provided       *
 *     tick and elapsed-tick callbacks             *
 *   - Compact fixed-size buffer for up to         *
 *     BLINK_BUFFER_COUNT simultaneous instances   *
 *                                                 *
 * @warning Ensure blink_initialize() is called    *
 * before using any other API, and provide valid   *
 * callback functions matching your platform.      *
 ***************************************************/


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
    uint32_t on_time_ms;
    uint32_t off_time_ms;
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

/**
 * @fn find_recorded_id
 * @brief Locate an existing or empty blink slot
 *
 * Searches the internal blink buffer for an entry whose `param` matches the
 * provided pointer. If found, returns its index. If not found, returns the
 * index of the first empty slot. If the buffer is full and no match exists,
 * returns -1.
 *
 * @param param  The pointer identifying a blink instance (as passed to blink_set).
 * @return
 *   >= 0 : Index into the blink buffer for the matching or empty slot  
 *   -1   : No matching entry and no empty slot available
 */
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

/**
 * @fn set_blink_state
 * @brief Update the on/off state of a blink instance and invoke the output callback
 *
 * This function looks up the internal blink buffer entry corresponding to
 * the given blink handle (`p_param`), updates its state flags and count,
 * and calls the user-provided output callback to drive the GPIO (or other)
 * resource to the new on/off level.
 *
 * @param p_param  Pointer to the blink instance (as registered via blink_set)
 * @param onoff    Desired output level: BLINK_ON (1) to turn on, or BLINK_OFF (0) to turn off
 */
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

/**
 * @fn blink_initialize
 * @brief Initialize the Blink Driver with user callbacks and time base
 *
 * Registers the platform-specific callback functions for driving the output,
 * obtaining the current tick count, and computing elapsed ticks. Also sets
 * the conversion factor from milliseconds to timer ticks.
 *
 * @param p_api
 *   Pointer to a blink_fp_api_t struct containing three callbacks:
 *   - fp_blink_output_func: drives the ON/OFF output for a blink instance  
 *   - fp_get_current_tick:   returns the current hardware timer tick count  
 *   - fp_tick_elapsed:       computes elapsed ticks between two tick values  
 * @param count_of_ticks_in_1ms
 *   Number of hardware ticks that correspond to 1 millisecond
 *
 * @return
 *   BLINK_INITIALIZE_SUCCESS (0) if all callbacks are valid and ticks_per_ms > 0,  
 *   BLINK_INITIALIZE_FAIL    (-1) otherwise
 */
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

/**
 * @fn blink_set
 * @brief Schedule a blink instance with specified timing parameters
 *
 * If the driver is initialized and a slot is available (or the identifier
 * already exists), this function sets up or updates a blink entry for the
 * given `param`. It initializes on/off durations, cycle count, and optional
 * start delay. If `offset` is zero, the first ON event is triggered immediately.
 *
 * @param param     User-defined identifier passed to the output callback
 * @param on_time_ms   Duration in milliseconds for which the resource stays ON
 * @param off_time_ms  The time in milliseconds that the source remains OFF, 
 * set to 0 for continuous ON cycles (until the cyle count is exhausted)
 * @param cycle     Number of ON/OFF toggles before the blink entry is freed
 * @param offset    Initial delay in milliseconds before the first ON event
 */
void blink_set(void * const param, const uint32_t on_time_ms, const uint32_t off_time_ms, const uint32_t cycle, const uint32_t offset)
{
    if (BLINK_INITIALIZE_SUCCESS == initialize_status)
    {
        int inx = find_recorded_id(param);
        if (inx >= 0)
        {
            blinks[inx].on_time_ms = on_time_ms;
            blinks[inx].off_time_ms = off_time_ms;
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

/**
 * @fn blink_process
 * @brief Drive all scheduled blink instances through their timing state machine
 *
 * This function must be called regularly (e.g., from the main loop or a periodic timer).
 * It iterates through each blink slot and:
 *   - Applies the initial offset delay before the first ON event
 *   - Toggles between ON and OFF according to `on_time_ms` and `off_time_ms`
 *   - Invokes the application callback for each state change
 *   - Counts each toggle and frees the slot when the cycle count is reached
 *
 * @note The driver must be successfully initialized (`blink_initialize`) before calling.
 */
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
                /* Handle initial offset delay */
                if (0 == (p_blink->states & BLINK_ENABLE_OFFSET_MASK))
                {
                    if (TICK_DIFF(p_blink->time, p_blink->offset * num_of_ticks_in_1ms))
                    {
                        p_blink->states |= BLINK_ENABLE_OFFSET_MASK;
                        p_blink->time = p_blink_fp_api->fp_get_current_tick();
                        set_blink_state(p_blink->param, BLINK_ON);
                    }
                }
                /* Determine whether currently ON or OFF */
                if (p_blink->states & BLINK_ENABLE_OFFSET_MASK)
                {
                    if (p_blink->states & BLINK_ON_OFF_STATE_MASK)
                    {
                        /* Currently ON: wait for on_time */
                        if (TICK_DIFF(p_blink->time, p_blink->on_time_ms * num_of_ticks_in_1ms))
                        {
                            p_blink->time = p_blink_fp_api->fp_get_current_tick();
                            if((0 != p_blink->cycle) && (p_blink->cycle >= p_blink->count))
                            {
                                set_blink_state(p_blink->param, (0 == p_blink->off_time_ms) ? BLINK_ON : BLINK_OFF);
                            }
                            else
                            {
                                /* Cycle complete: turn off and free slot */
                                p_blink->states &= ~BLINK_ENABLE_OFFSET_MASK;
                                set_blink_state(p_blink->param, BLINK_OFF);
                                p_blink->param = NULL;
                            }
                        }
                    }
                    else
                    {
                        /* Currently OFF: wait for off_time */
                        if (TICK_DIFF(p_blink->time, p_blink->off_time_ms * num_of_ticks_in_1ms))
                        {
                            p_blink->time = p_blink_fp_api->fp_get_current_tick();
                            if((0 != p_blink->cycle) && (p_blink->cycle >= p_blink->count))
                            {
                                set_blink_state(p_blink->param, BLINK_ON);
                            }
                            else
                            {
                                /* Cycle complete: free slot */
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

/**
 * @fn blink_remove
 * @brief Immediately stop and remove a scheduled blink instance
 *
 * Turns the output OFF for the blink instance associated with `param`,
 * clears its enable flag, and frees the slot so it can be reused.
 *
 * @param param  Identifier passed to blink_set for the instance to remove
 */
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

/**
 * @fn blink_get_state
 * @brief Retrieve the current active state of a blink instance
 *
 * Checks whether a blink entry associated with `param` is active (offset enabled).
 * Returns the enable flag mask if found and initialized, or -1 on error/not found.
 *
 * @param param  Identifier passed to blink_set for the instance to query
 * @return
 *   >= 0 : Non-zero if the blink instance is currently active (BLINK_ENABLE_OFFSET_MASK bit set),  
 *   0    : Found but not currently active,  
 *   -1   : Driver not initialized or no matching blink instance
 */
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

/**
 * @fn blink_force_update
 * @brief Immediately force an ON/OFF update without affecting cycle count
 *
 * Directly invokes the blink output callback for the instance identified by
 * `param`, setting it to the specified `onoff` level. This does not reset or
 * advance the cycle count—useful for manual overrides.
 *
 * @param param  Identifier previously passed to blink_set
 * @param onoff  Desired output level: BLINK_ON (1) or BLINK_OFF (0)
 */
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