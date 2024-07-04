#include "bridge.h"

void bridge_get_lower_resistance(resistance_t const* const upper_resistance, voltage_t const* const voltage_mv, voltage_t const* const vcc_mv,
                                 resistance_t* out_resistance)
{
    // Can't divide by 0, this is an error case
    if (*vcc_mv == *voltage_mv)
    {
        *out_resistance = UINT16_MAX;
        return;
    }

    uint32_t delta_t   = *vcc_mv - *voltage_mv;
    uint32_t numerator = ((uint32_t)*upper_resistance) * ((uint32_t)*voltage_mv);
    *out_resistance    = (resistance_t)(numerator / delta_t);
}