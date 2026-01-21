#ifndef TIMER_H
#define TIMER_H

#include "common_defines.h"
#include "system.h"

typedef struct timer_s 
{
    uint64_t wait_time;
    uint64_t target_time;
    bool timer_elapsed;
    bool auto_reset;
} timer_t;


/* Functions needed */
/**
 * - timer_init() - initialize timer
 * - timer_reset() - reset the timer 
 * - timer_timeout() - check if the timer has timed out
 *
 */

void timer_setup(timer_t* timer, uint64_t time, bool auto_reset);
void timer_reset(timer_t* timer);
bool timer_timeout(timer_t* timer);



#endif /* TIMER_H */