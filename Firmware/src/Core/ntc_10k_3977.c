#include "ntc_10k_3977.h"

const thermistor_data_t ntc_10k_3977_data =  {
    .data = {
        {0, 33900},
        {9, 21306},
        {19, 13151},
        {28, 8755},
        {38, 5727},
        {47, 3998},
        {57, 2744},
        {66, 1993},
        {76, 1425},
        {85, 1070}
    },
    .unit = RESUNIT_OHMS,
    .sample_count = NTC_10K_3977_SAMPLE_COUNT
};
