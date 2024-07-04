#include "interpolation.h"
#include <stdbool.h>

// Those aliasing factors are used to oversize a little
// bit temporary results in interpolation on integer, in order to reduce
// overall aliasing (the bigger the number the lesser the aliasing effect, so that we loose less "steps")
#define UINT16_ALIASING_FACTOR 100
#define UINT8_ALIASING_FACTOR 10

interpolation_range_check_t interpolation_check_value_range_uint16(uint16_t const *const value, range_uint16_t const *const range)
{
    bool positive_range = range->start < range->end;
    if (positive_range)
    {
        if (*value < range->start)
        {
            return RANGE_CHECK_LEFT;
        }

        if (*value > range->end)
        {
            return RANGE_CHECK_RIGHT;
        }
    }
    else
    {
        if (*value > range->start)
        {
            return RANGE_CHECK_LEFT;
        }

        if (*value < range->end)
        {
            return RANGE_CHECK_RIGHT;
        }
    }

    return RANGE_CHECK_INCLUDED;
}

interpolation_range_check_t interpolation_check_value_range_uint8(const uint8_t value, range_uint8_t const *const range)
{
    bool positive_range = range->start < range->end;
    if (positive_range)
    {
        if (value < range->start)
        {
            return RANGE_CHECK_LEFT;
        }

        if (value > range->end)
        {
            return RANGE_CHECK_RIGHT;
        }
    }
    else
    {
        if (value > range->start)
        {
            return RANGE_CHECK_LEFT;
        }

        if (value < range->end)
        {
            return RANGE_CHECK_RIGHT;
        }
    }

    return RANGE_CHECK_INCLUDED;
}

int8_t interpolation_linear_uint16_to_int8(uint16_t const *const value, range_uint16_t const *const in, range_int8_t const *const out)
{
    interpolation_range_check_t checked_value = interpolation_check_value_range_uint16(value, in);
    switch (checked_value)
    {
        case RANGE_CHECK_LEFT:
            // Clamp data to the start of output range
            return out->start;

        case RANGE_CHECK_RIGHT:
            // Clamp data to the end of output range
            return out->end;

        case RANGE_CHECK_INCLUDED:
        default:
            break;
    }

    int16_t in_delta  = in->end - in->start;
    int8_t  out_delta = out->end - out->start;

    // Note : using UINT16_ALIASING_FACTOR is a small trick to get back some resolution.
    // By nature, value is always contained within the range we need to check.
    // As a result, (*value - in->start) <= in_delta.
    // So dividing *value - in->start is always less than 1, hence loss in precision
    int16_t in_tmp = ((int16_t)*value - (int16_t)in->start);
    in_tmp *= UINT16_ALIASING_FACTOR;
    in_tmp /= in_delta;

    int16_t tmp_result = (in_tmp * (int16_t)out_delta);
    tmp_result /= UINT16_ALIASING_FACTOR;

    tmp_result += out->start;

    int8_t result = (int8_t)(tmp_result);

    return result;
}

uint8_t interpolation_linear_uint8_to_uint8(const uint8_t value, range_uint8_t const* const in, range_uint8_t const* const out)
{
    interpolation_range_check_t checked_value = interpolation_check_value_range_uint8(value, in);
    switch (checked_value)
    {
        case RANGE_CHECK_LEFT:
            // Clamp data to the start of output range
            return out->start;

        case RANGE_CHECK_RIGHT:
            // Clamp data to the end of output range
            return out->end;

        case RANGE_CHECK_INCLUDED:
        default:
            break;
    }

    uint8_t in_delta = in->end - in->start;
    uint8_t out_delta = out->end - out->start;

    uint16_t in_tmp = (uint16_t) (value - in->start);
    in_tmp *= UINT16_ALIASING_FACTOR;
    in_tmp /= (uint16_t) in_delta;

    uint16_t tmp_result = (in_tmp * out_delta);
    tmp_result /= UINT16_ALIASING_FACTOR;

    tmp_result += (uint16_t) out->start;

    uint8_t result = (uint8_t) tmp_result;
    return result;
}
