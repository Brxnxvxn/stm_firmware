#include "timer.h"

void timer_setup(timer_t* timer, uint64_t time, bool auto_reset)
{
    timer->auto_reset = auto_reset;
    timer->wait_time = time;
    timer->target_time = system_get_ticks() + time;
    timer->timer_elapsed = false;
}

void timer_reset(timer_t* timer)
{
    timer_init(timer, timer->wait_time, timer->auto_reset);
}

bool timer_timeout(timer_t* timer)
{
    uint64_t current_time = system_get_ticks();

    bool has_elapsed = false;

    if (timer->timer_elapsed)
        return true;

    if (current_time >= timer->target_time)
    {
        has_elapsed = true;

        /* check if auto reset is enabled */
        if (timer->auto_reset)
        {
            uint64_t extra_time = current_time - timer->target_time;
            timer->target_time = (current_time + timer->wait_time) - extra_time;
        }
        else
        {
            timer->timer_elapsed = true;
        }

    }

    return has_elapsed;
}