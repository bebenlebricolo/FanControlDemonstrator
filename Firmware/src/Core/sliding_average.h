#ifndef SLIDING_AVG_HEADER
#define SLIDING_AVG_HEADER

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

#define SLIDING_AVG_BUFFER_SIZE 10U

typedef struct
{
    int8_t data[SLIDING_AVG_BUFFER_SIZE];
    uint8_t index;
    uint8_t capacity;
    int16_t sum;
} sliding_avg_t;

void sliding_avg_push(sliding_avg_t * const buffer, const int8_t new_data, int8_t * const out);

#ifdef __cplusplus
}
#endif

#endif /* SLIDING_AVG_HEADER */