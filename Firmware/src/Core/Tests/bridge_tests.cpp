#include <gtest/gtest.h>

#include "bridge.h"
#include <vector>

class BridgeFixture : public ::testing::Test
{
protected:
    void SetUp() override {}
};

TEST_F(BridgeFixture, bridge_calculate_lower_resistance)
{
    resistance_t   upper_resistance = 330;
    voltage_t       voltage_milli    = 1500;
    const voltage_t vcc              = 5000;
    resistance_t       result           = 0;

    bridge_get_lower_resistance(&upper_resistance, &voltage_milli, &vcc, &result);
    ASSERT_EQ(result, 141);
    resistance_t expected = (vcc * result) / (upper_resistance + result);
    ASSERT_NEAR(voltage_milli, expected, 10);

    voltage_milli = 0;
    bridge_get_lower_resistance(&upper_resistance, &voltage_milli, &vcc, &result);
    ASSERT_EQ(result, 0);

    voltage_milli = vcc;
    bridge_get_lower_resistance(&upper_resistance, &voltage_milli, &vcc, &result);
    ASSERT_EQ(result, UINT16_MAX);
}

TEST_F(BridgeFixture, bridge_calculate_lower_resistance_multi)
{
    resistance_t      upper_resistance = 330;
    voltage_t          voltage_milli    = 1500;
    const voltage_t    vcc              = 5000;
    constexpr uint8_t STEPS            = 20;

    std::vector<std::pair<resistance_t, resistance_t>> results;
    for (uint8_t i = 0; i < STEPS; i++)
    {
        voltage_milli = (i * vcc / STEPS);
        auto& pair    = results.emplace_back();
        pair.first    = voltage_milli;
        bridge_get_lower_resistance(&upper_resistance, &voltage_milli, &vcc, &pair.second);
    }

    ASSERT_EQ(results[0].second, 0);
    ASSERT_EQ(results[(STEPS / 2)].second, upper_resistance);
    ASSERT_GT(results[STEPS - 1].second, 6000);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}