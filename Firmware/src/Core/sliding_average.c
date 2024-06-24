#include "sliding_average.h"

void sliding_avg_push(sliding_avg_t * const buffer, const int8_t new_data, int8_t * const out)
{
    if(buffer->capacity > 0)
    {
        buffer->sum -= buffer->data[buffer->index];
    }

    buffer->data[buffer->index] = new_data;
    buffer->sum += new_data;
    buffer->index++;
    buffer->index %= SLIDING_AVG_BUFFER_SIZE;

    if(buffer->capacity < SLIDING_AVG_BUFFER_SIZE)
    {
        buffer->capacity++;
    }

    *out = buffer->sum / buffer->capacity;
}
