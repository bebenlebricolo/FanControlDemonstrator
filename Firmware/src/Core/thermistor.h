// Thermistor header
#ifndef THERMISTOR_HEADER
#define THERMISTOR_HEADER

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "interpolation.h"
#include "common_types.h"

#define THERMISTOR_MAX_SAMPLES 50U


/**
 * @brief Encodes the actual scaler for the thermistor base resistance
*/
typedef enum
{
    RESUNIT_OHMS,       /**> Ohms               */
    RESUNIT_KILOOHMS,   /**> Kilo Ohms (10³)    */
    RESUNIT_MEGAOHMS    /**> Mega Ohms (10⁶)    */
} thermistor_resistance_unit_t;

/**
 * @brief Thermistor data base construct (maps a temperature with a resistance)
*/
typedef struct
{
    int8_t            temperature;  /**> Temperature data (ranging from -24°C usually to +25°C, 1 degree increment)                 */
    resistance_t      resistance;   /**> Resistance of the thermistor at the given temperature (unit given below, default is KOhms) */
} thermistor_temp_res_t;

/**
 * @brief Thermistor data for a given thermistor
*/
typedef struct {
    thermistor_temp_res_t data[THERMISTOR_MAX_SAMPLES]; /**> (Ordered) array of thermistor data pairing                                             */
    thermistor_resistance_unit_t unit;                  /**> Resistance scale used                                                                  */
    uint8_t sample_count;                    /**> Gives the actual sample count used for a single curve (can be less than maximum limit) */
} thermistor_data_t;


/**
 * @brief Converts input resistance reading to an actual temperature.
 * Data is linearly extrapolated when resistance value is out of bounds
 * otherwise it's linearly interpolated in between thermistor data points.
 * @param[in] thermistor : thermistor characteristic curve dataset
 * @param[in] resistance : NTC resistance as calculated from output voltage (resistor bridge with NTC)
 * @return the calculated (interpolated temperature).
 *          In case resistance reading is out of the curve's boundaries, temperature value will be clamped to the corresponding boundary (either max or min).
 * TODO : Implement out-of-bounds data extrapolation if the need arise get past the input data curve.
 *
*/
int8_t thermistor_read_temperature(thermistor_data_t const * const thermistor, resistance_t const * const resistance);

/**
 * @brief finds the enclosing range that contains the input resistance within the thermistor data curve.
 *
 * This iterates over the points (assuming the curve is monotonic, typical for a NTC) of the dataset and tries to frame the input resistance
 * value within known bounds.
 * In case no interval (frame) can be found, both low and high thermistor_temp_res_t pointers will point to one of the boundaries of the dataset (either min or max)
 * in order to indicate that the input resistance value is outside the exploitable range.
 *
 * For now, framing is performed using a linear search algorithm.
 * TODO : switch to a binary search instead to make things a bit more efficient.
 * @param[in]  thermistor : thermistor characteristic data curve
 * @param[in]  resistance : NTC resistance as calculated from output voltage (resistor bridge with NTC)
 * @param[out] low        : point to the lower resistance data point (lower resistance, hence higher temperature)
 * @param[out] high       : point to the higher resistance data point (higher resistance, hence lower temperature)
 * @return interpolation_range_check_t
 *      RANGE_CHECK_INCLUDED : input value is included within the two values range.
 *      RANGE_CHECK_LEFT     : input value is NOT included within the two values range, and is on the left side of the range.
 *      RANGE_CHECK_RIGHT    : input value is NOT included within the two values range, and is on the right side of the range.
*/
interpolation_range_check_t thermistor_frame_value(thermistor_data_t const * const thermistor, resistance_t const * const resistance, thermistor_temp_res_t const ** low, thermistor_temp_res_t const ** high);

#ifdef __cplusplus
}
#endif

#endif /* THERMISTOR_HEADER */