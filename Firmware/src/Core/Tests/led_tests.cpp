#include <cmath>

#include "led.h"
#include <gtest/gtest.h>
#include <vector>

class LedFixture : public ::testing::Test
{
protected:
    void SetUp() override
    {
        port = 0;
        leds[0] = led_io_t{
            .port = &port,
            .pin = 3,
        };
        leds[1] = led_io_t{
            .port = &port,
            .pin = 5,
        };

        time.milliseconds = 0;
        time.seconds = 0;
    }

    void TearDown() override
    {
        led_reset();
    }

    inline bool _led_is_on(const uint8_t led_id)
    {
        return (port & (1 << leds[led_id].pin)) != 0;
    }

    inline bool _led_is_off(const uint8_t led_id)
    {
        return (port & (1 << leds[led_id].pin)) == 0;
    }

    inline uint8_t _read_pin(const uint8_t led_id)
    {
        const uint8_t offset = leds[led_id].pin;
        return (port & (1 << offset)) >> offset;
    }

    inline uint16_t get_elapsed_time(const uint16_t start, const uint16_t end)
    {
        if (start > end)
        {
            return (end + 1000) - start;
        }
        return end - start;
    }

    led_io_t leds[2U];
    mcu_time_t time;
    volatile uint8_t port;
};

TEST_F(LedFixture, led_process_uninitialized_test)
{
    led_process(&time);
    // Should return early : init has not been called and no led was configured

    ASSERT_TRUE(_led_is_off(0));
    ASSERT_TRUE(_led_is_off(1));

    // Initialize internal memory
    led_init(leds, 2U);
}

TEST_F(LedFixture, led_process_initialized_pattern_none_test)
{
    // Initialize internal memory
    led_init(leds, 2U);

    led_process(&time);

    ASSERT_TRUE(_led_is_off(0));
    ASSERT_TRUE(_led_is_off(1));

    time.milliseconds = 12;
    led_process(&time);

    ASSERT_TRUE(_led_is_off(0));
    ASSERT_TRUE(_led_is_off(1));

    time.milliseconds = 25;
    led_process(&time);

    ASSERT_TRUE(_led_is_off(0));
    ASSERT_TRUE(_led_is_off(1));

    time.seconds = LED_BLINK_WARNING_HALF_P;
    led_process(&time);

    ASSERT_TRUE(_led_is_off(0));
    ASSERT_TRUE(_led_is_off(1));
}

// Testing out the Warning blink pattern
TEST_F(LedFixture, led_process_pattern_warning_test)
{
    // Initialize internal memory
    led_init(leds, 2U);

    led_set_blink_pattern(0U, LED_BLINK_WARNING);
    led_process(&time);

    // Led 1 should be HIGH now
    ASSERT_TRUE(_led_is_on(0));
    ASSERT_TRUE(_led_is_off(1));

    time.milliseconds = 12;
    led_process(&time);

    ASSERT_TRUE(_led_is_on(0));
    ASSERT_TRUE(_led_is_off(1));

    time.seconds = LED_BLINK_WARNING_HALF_P;
    led_process(&time);

    // Led should be off now
    ASSERT_TRUE(_led_is_off(0));
    ASSERT_TRUE(_led_is_off(1));

    time.seconds = LED_BLINK_WARNING_PERIOD_S;
    led_process(&time);

    // Led should be ON again now
    ASSERT_TRUE(_led_is_on(0));
    ASSERT_TRUE(_led_is_off(1));

    led_set_blink_pattern(0U, LED_BLINK_NONE);
    led_process(&time);

    // This should revert the led back to its off state :
    ASSERT_TRUE(_led_is_off(0));
    ASSERT_TRUE(_led_is_off(1));
}

TEST_F(LedFixture, led_process_pattern_accept_test)
{
    // Initialize internal memory
    led_init(leds, 1U);

    led_set_blink_pattern(0U, LED_BLINK_ACCEPT);
    led_process(&time);

    // ####################################
    // First pulse
    // ####################################

    // Led 1 should be HIGH now
    ASSERT_TRUE(_led_is_on(0));

    time.milliseconds = LED_BLINK_ACCEPT_ON_TIME_MS - 10;
    led_process(&time);
    ASSERT_TRUE(_led_is_on(0));

    time.milliseconds = LED_BLINK_ACCEPT_ON_TIME_MS;
    led_process(&time);
    ASSERT_TRUE(_led_is_off(0));

    // ####################################
    // Second pulse
    // ####################################

    time.milliseconds = LED_BLINK_ACCEPT_CYCLE_PERIOD_MS;
    led_process(&time);
    ASSERT_TRUE(_led_is_on(0));

    time.milliseconds = LED_BLINK_ACCEPT_CYCLE_PERIOD_MS + LED_BLINK_ACCEPT_ON_TIME_MS + 1U;
    led_process(&time);
    ASSERT_TRUE(_led_is_off(0));

    // ####################################
    // Third pulse
    // ####################################

    time.milliseconds = LED_BLINK_ACCEPT_CYCLE_PERIOD_MS * 2U;
    led_process(&time);
    ASSERT_TRUE(_led_is_on(0));

    time.milliseconds = (LED_BLINK_ACCEPT_CYCLE_PERIOD_MS * 2U) + LED_BLINK_ACCEPT_ON_TIME_MS + 1U;
    time.milliseconds %= 1000;
    led_process(&time);
    ASSERT_TRUE(_led_is_off(0));

    // ####################################
    // Now all subsequent calls should leave the LED off
    // ####################################

    time.milliseconds = 10;
    led_process(&time);
    ASSERT_TRUE(_led_is_off(0));

    time.milliseconds = LED_BLINK_ACCEPT_ON_TIME_MS;
    ASSERT_TRUE(_led_is_off(0));
}

TEST_F(LedFixture, led_breathing_get_duty_sawtooth_test)
{
    std::vector<uint8_t> duties;
    for (uint16_t i = 0; i < LED_BLINK_BREATHING_FULL_CYCLE_STEPS; i++)
    {
        if(i == LED_BLINK_BREATHING_HALF_CYCLE_STEPS)
        {
            i = LED_BLINK_BREATHING_HALF_CYCLE_STEPS -1 + 1 ;
        }
        uint8_t duty = led_breathing_get_duty_sawtooth(i);
        duties.push_back(duty);
    }

    ASSERT_EQ(duties[LED_BLINK_BREATHING_HALF_CYCLE_STEPS], 100U);
    ASSERT_EQ(duties[LED_BLINK_BREATHING_HALF_CYCLE_STEPS / 2], 50U);
    ASSERT_EQ(duties[LED_BLINK_BREATHING_HALF_CYCLE_STEPS * 3 / 2], 50U);
    ASSERT_EQ(duties[LED_BLINK_BREATHING_FULL_CYCLE_STEPS - 1], 1U);
    ASSERT_EQ(duties[0], 0U);
}

TEST_F(LedFixture, led_get_on_time_ticks_test)
{
    constexpr uint8_t resolution = 5U;  // Milliseconds, what we'd get with a 200 Hz frequency
    constexpr uint8_t duty_inc = 20U;   // Duty cycle increments per point of resolution

    uint8_t test = led_get_on_time_ticks(10, resolution, duty_inc);
    ASSERT_EQ(test, 1);

    std::vector<std::pair<uint8_t, uint8_t>> duty_on_time_map;
    for(uint8_t i = 0 ; i <= 100 ; i++)
    {
        auto& pair = duty_on_time_map.emplace_back();
        pair.second = led_get_on_time_ticks(i, resolution, duty_inc);
        pair.first = i;
    }

    ASSERT_EQ(duty_on_time_map.size(), 101);
}

#if defined(WIN32) && defined(_MSC_VER)
#pragma warning(push)
#pragma warning(disable : 4702) // Removes unreachable code
#endif
TEST_F(LedFixture, led_process_pattern_breathing_test)
{
    GTEST_SKIP() << "Test skipped as led breathing pattern test is not trivial to write (...)";

    // Initialize internal memory
    led_init(leds, 1U);

    led_set_blink_pattern(0U, LED_BLINK_BREATHING);
    led_process(&time);

    std::vector<uint16_t> on_times;

    // Collect data (4 seconds is a full wave cycle)
    uint8_t current_state = 0;
    uint8_t prev_state = 0;
    bool led_was_on = false;
    uint16_t last_milli = 0;
    uint16_t elapsed_on_time = 0;
    for (uint16_t i = 0; i < (LED_BLINK_BREATHING_PERIOD_S * 1000); i++)
    {
        time.milliseconds = i % 1000;

        led_process(&time);
        current_state = _read_pin(0);

        // Start of a new step and LED is ON
        if ((i % LED_BLINK_BREATHING_RESOLUTION_MS) == 0 && current_state == 1)
        {
            // track led on start time
            led_was_on = true;
        }

        // We are currently processing within a "step" (40 ms time window)
        if (led_was_on)
        {
            if ((i % LED_BLINK_BREATHING_RESOLUTION_MS) != 0)
            {
                // Falling edge detected
                if (current_state != prev_state && current_state == 0)
                {
                    elapsed_on_time = get_elapsed_time(last_milli, time.milliseconds);
                }
            }
        }

        // End of time window detected : reset trackers
        if ((i % LED_BLINK_BREATHING_RESOLUTION_MS) == (LED_BLINK_BREATHING_RESOLUTION_MS - 1))
        {
            // Handles the "always on" state
            if (led_was_on && current_state == 1)
            {
                elapsed_on_time = get_elapsed_time(last_milli, time.milliseconds);
            }
            on_times.push_back(elapsed_on_time);
            last_milli = time.milliseconds;
            elapsed_on_time = 0;
            led_was_on = false;
        }

        prev_state = current_state;
    }

    ASSERT_EQ(on_times[LED_BLINK_BREATHING_HALF_CYCLE_STEPS], LED_BLINK_BREATHING_RESOLUTION_MS / 2U);
}

#if defined(WIN32) && defined(_MSC_VER)
#pragma warning(pop)
#endif

int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}