#include <gtest/gtest.h>

#include "thermistor.h"

class ThermistorFixture : public ::testing::Test
{
protected:
    void SetUp() override
    {
    }

    static const thermistor_data_t thermistor_data;
};

const thermistor_data_t ThermistorFixture::thermistor_data = {
    .data = {
        {-24, 1353},
        {-19, 991},
        {-14, 734},
        {-9, 550},
        {-4, 416},
        {1, 318},
        {6, 246},
        {11, 192},
        {16, 151},
        {21, 119}
    },
    .unit = RESUNIT_KILOOHMS,
    .sample_count = 10
};

TEST_F(ThermistorFixture, thermistor_resistance_framing)
{
    uint16_t resistance = 1400;

    thermistor_temp_res_t const * low;
    thermistor_temp_res_t const * high;

    interpolation_range_check_t in_range = thermistor_frame_value(&thermistor_data, &resistance, &low, &high);
    ASSERT_EQ(in_range, RANGE_CHECK_LEFT);
    ASSERT_EQ(low, &thermistor_data.data[0]);
    ASSERT_EQ(high, low);

    resistance = 10;
    in_range = thermistor_frame_value(&thermistor_data, &resistance, &low, &high);
    ASSERT_EQ(in_range, RANGE_CHECK_RIGHT);
    ASSERT_EQ(low, &thermistor_data.data[thermistor_data.sample_count - 1]);
    ASSERT_EQ(high, low);

    resistance = 280;
    in_range = thermistor_frame_value(&thermistor_data, &resistance, &low, &high);
    ASSERT_EQ(in_range, RANGE_CHECK_INCLUDED);
    ASSERT_EQ(low, &thermistor_data.data[6]);
    ASSERT_EQ(high, &thermistor_data.data[5]);

    resistance = thermistor_data.data[7].resistance;
    in_range = thermistor_frame_value(&thermistor_data, &resistance, &low, &high);
    ASSERT_EQ(in_range, RANGE_CHECK_INCLUDED);
    ASSERT_EQ(low, &thermistor_data.data[7]);
    ASSERT_EQ(high, &thermistor_data.data[6]);

    // Checking at the very end of data as well
    resistance = thermistor_data.data[thermistor_data.sample_count - 1].resistance;
    in_range = thermistor_frame_value(&thermistor_data, &resistance, &low, &high);
    ASSERT_EQ(in_range, RANGE_CHECK_INCLUDED);
    ASSERT_EQ(low, &thermistor_data.data[9]);
    ASSERT_EQ(high, &thermistor_data.data[9]);

    resistance = thermistor_data.data[0].resistance;
    in_range = thermistor_frame_value(&thermistor_data, &resistance, &low, &high);
    ASSERT_EQ(in_range, RANGE_CHECK_INCLUDED);
    ASSERT_EQ(low, &thermistor_data.data[0]);
    ASSERT_EQ(high, &thermistor_data.data[0]);
}

TEST_F(ThermistorFixture, thermistor_read_temperature)
{
    uint16_t resistance = 2000;

    // Testing really out of bounds
    int8_t temp = thermistor_read_temperature(&thermistor_data, &resistance);
    ASSERT_EQ(temp, thermistor_data.data[0].temperature);

    resistance = 10;
    temp = thermistor_read_temperature(&thermistor_data, &resistance);
    ASSERT_EQ(temp, thermistor_data.data[thermistor_data.sample_count - 1].temperature);

    // Testing with edge values
    resistance = thermistor_data.data[0].resistance;
    temp = thermistor_read_temperature(&thermistor_data, &resistance);
    ASSERT_EQ(temp, thermistor_data.data[0].temperature);

    resistance = thermistor_data.data[thermistor_data.sample_count - 1].resistance;
    temp = thermistor_read_temperature(&thermistor_data, &resistance);
    ASSERT_EQ(temp, thermistor_data.data[thermistor_data.sample_count - 1].temperature);

    // Now with sensible values
    resistance = 280;
    temp = thermistor_read_temperature(&thermistor_data, &resistance);
    ASSERT_EQ(temp, 4);

    resistance = 500;
    temp = thermistor_read_temperature(&thermistor_data, &resistance);
    ASSERT_EQ(temp, -7);

    resistance = 770;
    temp = thermistor_read_temperature(&thermistor_data, &resistance);
    ASSERT_EQ(temp, -14);

    resistance = 790;
    temp = thermistor_read_temperature(&thermistor_data, &resistance);
    ASSERT_EQ(temp, -15);
}


int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}