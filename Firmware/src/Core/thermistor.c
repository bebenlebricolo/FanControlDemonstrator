#include "thermistor.h"
#include "interpolation.h"

#include <stddef.h>

interpolation_range_check_t thermistor_frame_value(thermistor_data_t const * const thermistor, resistance_t const * const resistance, thermistor_temp_res_t const ** low, thermistor_temp_res_t const ** high)
{
    // Clamp resistance to the lowest resistance found in the curve (aka highest temperature)
    if(*resistance < thermistor->data[thermistor->sample_count - 1].resistance)
    {
        *low = &(thermistor->data[thermistor->sample_count - 1]);
        *high = *low;
        return RANGE_CHECK_RIGHT;
    }

    // Clamp resistance to the highest resistance found in the curve (aka lowest temperature)
    if(*resistance > thermistor->data[0].resistance)
    {
        *low = &(thermistor->data[0]);
        *high = *low;
        return RANGE_CHECK_LEFT;
    }

    if(*resistance == thermistor->data[0].resistance)
    {
        *low = &(thermistor->data[0]);
        *high = *low;
        return RANGE_CHECK_INCLUDED;
    }

    if(*resistance == thermistor->data[thermistor->sample_count - 1].resistance)
    {
        *low = &(thermistor->data[thermistor->sample_count - 1]);
        *high = *low;
        return RANGE_CHECK_INCLUDED;
    }

    // Linear search
    *low = &(thermistor->data[thermistor->sample_count - 1]);
    *high = &(thermistor->data[0]);
    for(uint8_t i = 0 ; i < thermistor->sample_count; i++)
    {
        if(*resistance >= thermistor->data[i].resistance)
        {
            *low = &(thermistor->data[i]);
            *high = &(thermistor->data[i - 1]);
            break;
        }
    }

    return RANGE_CHECK_INCLUDED;
}

int8_t thermistor_read_temperature(thermistor_data_t const * const thermistor, resistance_t const * const resistance)
{
    int8_t result = 0;

    // Find boundaries of resistance within the lookup table
    thermistor_temp_res_t const * low = NULL;
    thermistor_temp_res_t const * high = NULL;

    interpolation_range_check_t check = thermistor_frame_value(thermistor, resistance, &low, &high);
    switch(check)
    {
        case RANGE_CHECK_LEFT :
            return low->temperature;

        case RANGE_CHECK_RIGHT :
            return high->temperature;

        case RANGE_CHECK_INCLUDED:
        default:
            // Happens when resistance value is outside boundaries or exactly sitting on one end value
            // of input range.
            if(low == high)
            {
                return low->temperature;
            }
            break;
    }


    range_int8_t temp_range = {
        .start = low->temperature,
        .end = high->temperature
    };
    range_uint32_t res_range = {
        .start = low->resistance,
        .end = high->resistance
    };

    // Then linearly interpolate the value within the boundaries
    result = interpolation_linear_uint32_to_int8(resistance, &res_range, &temp_range);
    return result;
}