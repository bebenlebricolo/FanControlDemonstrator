#ifndef INTERPOLATION_HEADER
#define INTERPOLATION_HEADER

#ifdef __cplusplus
extern "C"
{
#endif

#include<stdint.h>

/**
 * @brief Enum used to check if a value is contained within a range or not.
 * Note that we use the "Right" or "Left" convention, as this method can adapt to ranges which are reversed (start value is greater than end value).
 * This is done this way in order to support decreasing curves as well as increasing curves.
 *
*/
typedef enum
{
    RANGE_CHECK_INCLUDED,   /**> Value is included within the range             */
    RANGE_CHECK_LEFT,       /**> Value is outside the range, before its start   */
    RANGE_CHECK_RIGHT       /**> Value is outside the range, past the end       */
} interpolation_range_check_t;

/**
 * @brief Numerical range with uint16_t arithmetic
 * Note that ranges can be reversed (start is bigger than end, decreasing range)
*/
#define MAKE_RANGE(type) \
    typedef struct          \
    {                       \
        type start;     \
        type end;       \
    } range_##type;

MAKE_RANGE(uint8_t);
MAKE_RANGE(int8_t);
MAKE_RANGE(uint16_t);
MAKE_RANGE(int16_t);
MAKE_RANGE(uint32_t);
MAKE_RANGE(int32_t);
MAKE_RANGE(size_t);

/**
 * @brief interpolates the value within the input range into the output range.
 * @param[in] value         : the value that needs to be converted in the output range
 * @param[in] input_range   : input value range
 * @param[in] output_range  : output value range
 * @return int8_t : input value mapped in the output range
*/
int8_t interpolation_linear_uint16_to_int8(uint16_t const * const value,
                                           range_uint16_t const * const input_range,
                                           range_int8_t const * const out_range );


/**
 * @brief interpolates the value within the input range into the output range.
 * @param[in] value         : the value that needs to be converted in the output range
 * @param[in] input_range   : input value range
 * @param[in] output_range  : output value range
 * @return int8_t : input value mapped in the output range
*/
uint8_t interpolation_linear_uint8_to_uint8(const uint8_t value,
                                            range_uint8_t const * const input_range,
                                            range_uint8_t const * const out_range );

/**
 * @brief Checks whether an input value is contained within the input range (value pair) or not.
 * This function uses the included version of the check : [val1, val2] instead of the excluding check ]val1,val2[.
 * In case the input value is outside of the given range, returned enum indicates on which side the value is located.
 * @return
 *      RANGE_CHECK_INCLUDED : input value is included within the two values range.
 *      RANGE_CHECK_LEFT     : input value is NOT included within the two values range, and is on the left side of the range.
 *      RANGE_CHECK_RIGHT    : input value is NOT included within the two values range, and is on the right side of the range.
*/
interpolation_range_check_t interpolation_check_value_range_uint16(uint16_t const * const value, range_uint16_t const * const range);

/**
 * @brief Checks whether an input value is contained within the input range (value pair) or not.
 * This function uses the included version of the check : [val1, val2] instead of the excluding check ]val1,val2[.
 * In case the input value is outside of the given range, returned enum indicates on which side the value is located.
 * @return
 *      RANGE_CHECK_INCLUDED : input value is included within the two values range.
 *      RANGE_CHECK_LEFT     : input value is NOT included within the two values range, and is on the left side of the range.
 *      RANGE_CHECK_RIGHT    : input value is NOT included within the two values range, and is on the right side of the range.
*/
interpolation_range_check_t interpolation_check_value_range_uint8(const uint8_t value, range_uint8_t const * const range);


#ifdef __cplusplus
}
#endif

#endif /* INTERPOLATION_HEADER */