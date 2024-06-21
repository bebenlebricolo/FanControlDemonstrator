#include "ntc_10k_3977.h"

const thermistor_data_t ntc_10k_3977_data =  {
    .data = {
        {-24, 137},
        {-19, 100},
        {-13, 70},
        {-8, 52},
        {-2, 37},
        {3, 28},
        {9, 21},
        {14, 16},
        {20, 12},
        {25, 10}
    },
    .unit = RESUNIT_KILOOHMS,
    .sample_count = NTC_10K_3977_SAMPLE_COUNT
};
