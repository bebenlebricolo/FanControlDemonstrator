#include "mcu_time.h"

void time_default(mcu_time_t * time)
{
    time->milliseconds = 0;
    time->seconds = 0;
}

time_compare_t time_compare(const mcu_time_t *time_1, const mcu_time_t *time_2 )
{
    if(time_1->seconds < time_2->seconds)
    {
        return TIME_COMP_LT;
    }

    if(time_1->seconds > time_2->seconds)
    {
        return TIME_COMP_GT;
    }

    // Seconds are Equals

    if(time_1->milliseconds < time_2->milliseconds)
    {
        return TIME_COMP_LT;
    }

    if(time_1->milliseconds > time_2->milliseconds)
    {
        return TIME_COMP_GT;
    }

    return TIME_COMP_EQUAL;
}

void time_duration(const mcu_time_t *time_1, const mcu_time_t *time_2 , mcu_time_t * const output)
{
    time_compare_t comparison = time_compare(time_1, time_2);

    if(TIME_COMP_EQUAL == comparison)
    {
        output->milliseconds = 0;
        output->seconds = 0;
        return;
    }

    if(TIME_COMP_GT == comparison)
    {
        output->milliseconds = time_1->milliseconds - time_2->milliseconds;
        output->seconds = time_1->seconds - time_2->seconds;
    }
    else
    {
        output->milliseconds = time_2->milliseconds - time_1->milliseconds;
        output->seconds = time_2->seconds - time_1->seconds;
    }
}


void time_to_ms(const mcu_time_t* time, uint32_t* const output_ms)
{
    *output_ms = time->milliseconds + 1000 * time->seconds;
}

void time_duration_ms(const mcu_time_t *time_1, const mcu_time_t *time_2 , uint32_t* output_ms)
{
    mcu_time_t output = {0};
    time_duration(time_1, time_2, &output);
    time_to_ms(&output, output_ms);
}
