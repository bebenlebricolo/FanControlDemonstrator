#ifndef TIME_HEADER
#define TIME_HEADER

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/**
 * @brief helps keeping track of time
 */
typedef struct
{
    uint32_t seconds;      /**> Counts seconds         */
    uint16_t milliseconds; /**> Counts milliseconds    */
} mcu_time_t;

/**
 * @brief resets time construct to default. (All zeros)
 * @param[in/out] time : time construct to be reset
 */
void time_default(mcu_time_t* time);

typedef enum
{
    TIME_COMP_EQUAL,
    TIME_COMP_GT,
    TIME_COMP_LT
} time_compare_t;

time_compare_t time_compare(const mcu_time_t* time_1, const mcu_time_t* time_2);
void time_to_ms(const mcu_time_t* time, uint32_t* const output_ms);
void time_duration(const mcu_time_t* time_1, const mcu_time_t* time_2, mcu_time_t* const output);
void time_duration_ms(const mcu_time_t* time_1, const mcu_time_t* time_2, uint32_t* const output_ms);

#ifdef __cplusplus
}
#endif

#endif /* TIME_HEADER */