#include <Arduino.h>
#include <stdbool.h>
#include <stdint.h>

#include "Core/bridge.h"
#include "Core/buffers.h"
#include "Core/buttons.h"
#include "Core/current.h"
#include "Core/mcu_time.h"
#include "Core/thermistor.h"
#include "Core/ntc_10k_3977.h"
#include "Core/sliding_average.h"

#include "Core/led.h"

#include "Hal/persistent_memory.h"
#include "Hal/timebase.h"

// clang-format off
// -> Clang has troubles treating multiline macros comments
// https://github.com/llvm/llvm-project/issues/54399

// ################################################################################################################################################
// ################################################### Debugging defines and flags ################################################################
// ################################################################################################################################################

#define NTC_RESISTOR_MULTIPLICATOR 10U

#define DEBUG_SERIAL
#define DEBUG_TEMP 1
#define DEBUG_POT 0
#define DISABLE_MOTOR
//#define
//#define DEBUG_CURRENT_VOLTAGE

//#define CURRENT_LED_DEBUG
#define DEBUG_REPORT_PERIODIC 1
#if DEBUG_REPORT_PERIODIC == 1
    #define DEBUG_REPORT_PERIOD_SECONDS 1U
    #define DEBUG_REPORT_PERIOD_SECONDS_MOTOR_RESTART_ETA 10U
#endif
#define FORCE_OVERWRITE_EEPROM 0

#ifdef DEBUG_SERIAL
    #define MSG_LENGTH 50U
    char msg[MSG_LENGTH] = {0};
    #define LOG_INIT() Serial.begin(9600)
    #define LOG(msg) Serial.print(msg)
    #define LOG_CUSTOM(format, ...)                     \
        snprintf(msg, MSG_LENGTH, format, __VA_ARGS__); \
        LOG(msg);
#else
    #define LOG_INIT()
    #define LOG(msg)
    #define LOG_CUSTOM(format, ...)
#endif

// clang-format on

// ################################################################################################################################################
// ########################################################## Other Constants #####################################################################
// ################################################################################################################################################

// Pin mapping
const uint8_t motor_control_pin = 5; // D5
const uint8_t status_led_pin_1  = 7; // D7
const uint8_t status_led_pin_2  = 6; // D6

const uint8_t  button_pin         = 3; // D3
const uint8_t  temp_sensor_pin    = A3;     /**> NTC thermistor temperature sensor input pin             */
const uint8_t  potentiometer_pin  = A2;     /**> Current sens input pin, sampled @ 500Hz - 1kHz          */
const uint32_t upper_resistance   = 10000U; /**> 10 kOhms resistor is used as the upper bridge resistor */
const uint16_t vcc_mv             = 5000U;  /**> Board is powered via USB -> 5V                          */

// ################################################################################################################################################
// ################################################### Application state machine ##################################################################
// ################################################################################################################################################

/**
 * @brief controls the application state (state machine)
 */
typedef enum
{
    APP_STATE_TEMP_CONTROL,     /**> Regular temperature controlled scheme                  */
    APP_STATE_MANUAL_CONTROL,   /**> Uses the potentiometer as the source of input data     */
    APP_STATE_FULL_STEAM,       /**> Motor output is set to 100%                            */
    APP_STATE_DISABLED          /**> Motor is disabled                                      */
} app_state_t;

typedef enum
{
    LED_GREEN,
    LED_RED,
    LED_OFF
} led_state_t;

#define APP_STATE_ENUM_COUNT 4U

typedef struct
{
    app_state_t app_state; /**> Tracks application current state                                               */

    /**
     * @brief Keeps track of button events (either Pressed, Released, or Hold)
     */
    struct
    {
        // We need to detect transistion from the PRESSED event to the HOLD event exactly once
        button_state_t now;      /**> Minus button event                                                                                  */
        button_state_t prev; /**> Used to detect the change in button event (detecting from Pressed to Hold and from Hold to Release) */
    } button;

} app_working_mem_t;

typedef struct
{
    int8_t   temperature;
    uint16_t current_ma;
    uint16_t current_rms;
} app_sensors_t;

// ####################################################### Static declarations ####################################################################
// ################################################################################################################################################
// ################################################################################################################################################

static void read_button_event(button_state_t* const event, const mcu_time_t* time);
static void set_motor_output(const uint8_t value);
static void read_temperature(const mcu_time_t* time, int8_t* temperature);
static void read_potentiometer(const mcu_time_t* time, uint16_t* value);
static void set_leds_states(const led_state_t state);

#ifndef NO_CURRENT_MONITORING
static app_state_t handle_motor_stalled_loop(uint32_t const* const start_time, const mcu_time_t* time);
static void        read_current(const mcu_time_t* time, int16_t* current_ma, int16_t* current_rms_ma);
#endif

#ifdef DEBUG_CURRENT_VOLTAGE
static circular_buffer_t voltage_buffer;
#endif

void setup()
{
    pinMode(button_pin, INPUT);
    pinMode(temp_sensor_pin, INPUT);
    pinMode(potentiometer_pin, INPUT);

    pinMode(motor_control_pin, OUTPUT);
    pinMode(status_led_pin_1, OUTPUT);
    pinMode(status_led_pin_2, OUTPUT);

    timebase_init();

    LOG_INIT();
    sei();

#ifdef DEBUG_CURRENT_VOLTAGE
    circular_buffer_init(&voltage_buffer, 0);
#endif
}

void loop()
{
    static const mcu_time_t* time        = NULL;
    static int8_t            temperature = 0;
    static uint8_t           motor_pwm   = 0U; // Motor PWM (as per analogWrite) ranges from 0 to 255
    static uint16_t          pot_value   = 0U;

#ifdef DEBUG_REPORT_PERIODIC
    static mcu_time_t previous_time;
#endif

    // Keeps track of the previous time the system was toggled
    static app_working_mem_t app_mem = {
        .app_state = APP_STATE_TEMP_CONTROL,
        .button =
            {
                .now  = BUTTON_STATE_RELEASED,
                .prev = BUTTON_STATE_RELEASED,
            }, // Trailing comma is used to work around clang-format issues with struct fields initialization formatting
    };

    // Very important to process the current time as fast as we can (polling mode)
    timebase_process();
    time = timebase_get_time();

    // Temperature is read once every 2 seconds
    read_temperature(time, &temperature);

    // Potentiometer is read at every loop
    read_potentiometer(time, &pot_value);

    // Process button events.
    // Used to trigger
    read_button_event(&app_mem.button.now, time);

    // User pressed and release the + button.
    // Raise temp set point by one degree
    if (app_mem.button.now == BUTTON_STATE_PRESSED && app_mem.button.prev != app_mem.button.now)
    {
        app_mem.app_state = (app_state_t)((app_mem.app_state + 1) % APP_STATE_ENUM_COUNT);
        LOG_CUSTOM("Button Clicked ! Switching to mode %d\n", (uint8_t)app_mem.app_state);
    }


    switch (app_mem.app_state)
    {
        case APP_STATE_TEMP_CONTROL:
            // linear ramp between 20°C and 40°C ; value is clamped to min and max.
            if(temperature >= 20)
            {
                range_uint8_t input_range = {
                    .start = 31,
                    .end = 40
                };
                range_uint8_t output_range = {
                    .start = 0,
                    .end = UINT8_MAX
                };
                motor_pwm = interpolation_linear_uint8_to_uint8((uint8_t) temperature, &input_range, &output_range);
                set_leds_states(LED_RED);
            }
            else
            {
                motor_pwm = 0;
            }
            break;

        case APP_STATE_MANUAL_CONTROL:
            set_leds_states(LED_GREEN);
            motor_pwm =  pot_value >> 2U;
            break;

        case APP_STATE_FULL_STEAM:
            set_leds_states(LED_OFF);
            motor_pwm = UINT8_MAX;
            break;

        case APP_STATE_DISABLED:
        default: {
            set_leds_states(LED_OFF);
            motor_pwm = 0;
            break;
        }
    }

#ifndef DISABLE_MOTOR
    set_motor_output(motor_pwm);
#else
    set_motor_output(0);
#endif
    app_mem.button.prev  = app_mem.button.now;

#if DEBUG_REPORT_PERIODIC == 1
    if ((time->seconds - previous_time.seconds) > DEBUG_REPORT_PERIOD_SECONDS)
    {
        // Report few things about current states
        previous_time = *time;
        LOG_CUSTOM("temperature : %hd °C\n", temperature);
        LOG_CUSTOM("Current motor_pwm %u\n", motor_pwm);

#ifdef DEBUG_CURRENT_VOLTAGE
        for (uint8_t i = 0; i < CURRENT_MEASURE_SAMPLES_PER_SINE; i++)
        {
            LOG_CUSTOM("Voltage/current data (mv) [%hu] = %hd\n", i, voltage_buffer.data[i]);
        }
#endif
    }
#endif
}

static void set_leds_states(const led_state_t state)
{
    switch(state)
    {
        case LED_RED:
            digitalWrite(status_led_pin_1, HIGH);
            digitalWrite(status_led_pin_2, LOW);
            break;

        case LED_GREEN:
            digitalWrite(status_led_pin_1, LOW);
            digitalWrite(status_led_pin_2, HIGH);
            break;

        case LED_OFF:
        default:
            digitalWrite(status_led_pin_1, LOW);
            digitalWrite(status_led_pin_2, LOW);
            break;
    }
}


static void read_button_event(button_state_t* const event, const mcu_time_t* time)
{
    //     5V
    //    ┌┴┐
    //    │ │
    //    └┬┘
    //   ┌─┴─┐┄┄┄┄┄┄┄┄ Input line to Arduino
    //  ─┴─  O │
    //  ─┬─    ├─ <----- Switch (push button)
    //   │   O │
    //   └─┬─┘
    //     ╧ GND

    static button_local_mem_t button_mem = {
        .current  = HIGH,       // Starts up as a high state (button is not pressed, hence no current flows through it)
        .previous = HIGH,
        .pressed  = 0,
        .event    = BUTTON_STATE_RELEASED,
    };

    // Need to check buttons every 20 milliseconds (50Hz)
    static mcu_time_t last_checked = {0};
    static const mcu_time_t time_check_interval {
        .seconds = 0,
        .milliseconds = 20
    };

    // Compute how it's been since we last checked the buttons states
    mcu_time_t diff = {0};
    time_duration(time, &last_checked, &diff);
    if(time_compare(&diff, &time_check_interval) == TIME_COMP_GT)
    {
        last_checked = *time;

        button_mem.current = digitalRead(button_pin);

        read_single_button_event(&button_mem, &time->seconds);
        *event = button_mem.event;
    }
}


void set_motor_output(const uint8_t value)
{
    analogWrite(motor_control_pin, (int) value);
}

static void read_temperature(const mcu_time_t* time, int8_t* temperature)
{
    static uint32_t last_check_s = 0;
    static sliding_avg_t avg = {0};

    // Only trigger temperature reading if elapsed time is greater than 1 second.
    if (time->seconds - last_check_s >= 1U)
    {
        uint16_t temp_reading_raw = analogRead(temp_sensor_pin);
        last_check_s              = time->seconds;

        // Using the x10 to lower aliasing but still retain a more accurate
        // millivolt reading Also, this remains right under the overflow : (5000 x 10 < UINT16_MAX)
        uint16_t temp_reading_mv = (((vcc_mv * 10U) / 1024) * temp_reading_raw) / 10U;

        uint32_t ntc_resistance = 0;
        bridge_get_lower_resistance(&upper_resistance, &temp_reading_mv, &vcc_mv, &ntc_resistance);

        int8_t temp_reading = thermistor_read_temperature(&ntc_10k_3977_data, &ntc_resistance);
        sliding_avg_push(&avg, temp_reading, temperature);

#if DEBUG_TEMP
        LOG_CUSTOM("Temp mv : %u mV\n", temp_reading_mv)
        LOG_CUSTOM("Vcc mv : %u mV\n", vcc_mv)
        LOG_CUSTOM("Upper resistance : %lu k\n", upper_resistance)
        LOG_CUSTOM("Temperature : %d °C\n\n", (int)*temperature)
        LOG_CUSTOM("NTC res : %lu k\n", ntc_resistance)
        LOG_CUSTOM("Temp raw : %u /1024\n", temp_reading_raw)
#endif
    }
}

static void read_potentiometer(const mcu_time_t* time, uint16_t* value)
{
    // The potentiometer itself is wired backwards !
    *value = 1023 - (uint16_t) analogRead(potentiometer_pin);
#if DEBUG_POT
        LOG_CUSTOM("Pot value : %u\n", *value);
#endif
}

#ifndef NO_CURRENT_MONITORING
static void read_current(const mcu_time_t* time, int16_t* current_ma, int16_t* current_rms_ma)
{
    static uint16_t last_check_ms = 0;

    // Only trigger temperature reading if elapsed time is greater than 20 millisecond (for 50Hz).
    if (time->milliseconds - last_check_ms >= (1000 / (CURRENT_MEASURE_SAMPLES_PER_SINE * MAINS_AC_FREQUENCY_HZ)))
    {
        uint16_t current_raw = analogRead(current_sensor_pin);
        last_check_ms        = time->milliseconds;
#ifdef CURRENT_LED_DEBUG
        PORTD ^= (1 << PORTD3);
#endif
        int16_t current_reading_mv = (((vcc_mv * 10U) / 1024) * current_raw) / 10U;
        // LOG_CUSTOM("Current reading from ADC (mv) : %d\n", current_reading_mv);

        // Remove the DC part of the read current, as the opamp output is still polarized to vcc_mv/2
        current_reading_mv -= CURRENT_SENSE_DC_BIAS_MV;
#ifdef DEBUG_CURRENT_VOLTAGE
        circular_buffer_push_back(&voltage_buffer, current_reading_mv);
        LOG_CUSTOM("Current reading mv - DC part : %d\n", current_reading_mv);
#endif

        current_from_voltage(&current_reading_mv, current_ma);
        // LOG_CUSTOM("Current reading from ADC (ma) : %d\n", *current_ma);

#if CURRENT_RMS_ARBITRARY_FCT == 1
        int16_t dc_offset_current = 300;
        current_compute_rms_arbitrary(&current_ma, &current_rms, &dc_offset_current);
#else
        // Takes care about the remaining DC part, current_ma should still have this DC component otherwise RMS won't work.
        current_compute_rms_sine(current_ma, current_rms_ma);
#endif /* CURRENT_RMS_ARBITRARY_FCT */

        // LOG_CUSTOM("Current RMS reading (ma) : %d\n", *current_rms_ma);
    }
}
#endif /* NO_CURRENT_MONITORING */