#ifndef BRIDGE_HEADER
#define BRIDGE_HEADER

#ifdef __cplusplus
extern "C" {
#endif

#include "common_types.h"
#include <stdint.h>

/**
 * @brief computes lower bridge resistance based on upper resistance value and voltage (milli volt)
 * @param[in] upper_resistance : upper bridge resistance value (don't need to specify unit here)
 * @param[in] voltage_mv       : voltage reading above the lower resistance of the bridge (in millivolt)
 * @param[in] vcc_mv           : input vcc voltage of the resistor bridge (in millivolt)
 * @param[out] out_resistance  : output calculated resistance (same unit as input resistance)
 */
void bridge_get_lower_resistance(resistance_t const* const upper_resistance, voltage_t const* const voltage_mv, voltage_t const* const vcc_mv,
                                 resistance_t* out_resistance);

#ifdef __cplusplus
}
#endif

#endif /* BRIDGE_HEADER */