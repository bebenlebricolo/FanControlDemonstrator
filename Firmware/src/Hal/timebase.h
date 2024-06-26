#ifndef TIMEBASE_HEADER
#define TIMEBASE_HEADER

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>
#include "Core/mcu_time.h"

/**
 * @brief initialises timebase and starts timer 2
*/
void timebase_init(void);

/**
 * @brief needs to be called at each loop cycle in order to safely keep track
 * of the time
*/
void timebase_process(void);

/**
 * @brief Retrieves current time (readonly memory)
*/
const mcu_time_t *  timebase_get_time(void);

/**
 * @brief resets internal time back to 0 and reverts timer 2 to its original condition.
*/
void timebase_reset(void);


#ifdef __cplusplus
}
#endif

#endif /* TIMEBASE_HEADER */