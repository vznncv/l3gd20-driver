#include "greentea-client/test_env.h"
#include "l3gd20_driver.h"
#include "math.h"
#include "mbed.h"
#include "rtos.h"
#include "unity.h"
#include "utest.h"

using namespace utest::v1;
using namespace l3gd20;

static L3GD20Gyroscope *gyro;

utest::v1::status_t test_setup_handler(const size_t number_of_cases)
{
    gyro = new L3GD20Gyroscope(MBED_CONF_L3GD20_DRIVER_TEST_SPI_MOSI, MBED_CONF_L3GD20_DRIVER_TEST_SPI_MISO, MBED_CONF_L3GD20_DRIVER_TEST_SPI_SCLK, MBED_CONF_L3GD20_DRIVER_TEST_SPI_CS);
    return greentea_test_setup_handler(number_of_cases);
}

void test_teardown_handler(const size_t passed, const size_t failed, const failure_t failure)
{
    delete gyro;
    return greentea_test_teardown_handler(passed, failed, failure);
}

utest::v1::status_t case_setup_handler(const Case *const source, const size_t index_of_case)
{
    // reset gyroscope
    int err = gyro->init();
    // wait to skip noisy data after device enabling
    ThisThread::sleep_for(100ms);

    utest::v1::status_t gt_status = greentea_case_setup_handler(source, index_of_case);
    if (err) {
        gt_status = STATUS_ABORT;
    }

    return gt_status;
}

/**
 * Test gyroscope state after initialization.
 */
void test_init_state_enabled()
{
    int err;

    // check that init isn't failed
    err = gyro->init();
    TEST_ASSERT_EQUAL(0, err);

    // check parameter
    TEST_ASSERT_EQUAL(L3GD20Gyroscope::DRDY_DISABLE, gyro->get_data_ready_interrupt_mode());
    TEST_ASSERT_EQUAL(L3GD20Gyroscope::FIFO_DISABLE, gyro->get_fifo_mode());
    TEST_ASSERT_EQUAL(0, gyro->get_fifo_watermark());
    TEST_ASSERT_EQUAL(L3GD20Gyroscope::HPF_DISABLE, gyro->get_high_pass_filter_mode());
    TEST_ASSERT_EQUAL(L3GD20Gyroscope::HPF_CF0, gyro->get_high_pass_filter_cutoff_freq_mode());
    TEST_ASSERT_EQUAL(L3GD20Gyroscope::LPF_CF0, gyro->get_low_pass_filter_cutoff_freq_mode());
    TEST_ASSERT_EQUAL(L3GD20Gyroscope::ODR_95_HZ, gyro->get_output_data_rate());
    TEST_ASSERT_EQUAL(L3GD20Gyroscope::DRDY_DISABLE, gyro->get_sensitivity_dps());
    TEST_ASSERT_EQUAL(L3GD20Gyroscope::DRDY_DISABLE, gyro->get_data_ready_interrupt_mode());
    TEST_ASSERT_EQUAL(L3GD20Gyroscope::G_ENABLE, gyro->get_gyroscope_mode());
}

/**
 * Test gyroscope state after initialization.
 */
void test_init_state_disabled()
{
    int err;

    // check that init isn't failed
    err = gyro->init(false);
    TEST_ASSERT_EQUAL(0, err);

    // check parameter
    TEST_ASSERT_EQUAL(L3GD20Gyroscope::DRDY_DISABLE, gyro->get_data_ready_interrupt_mode());
    TEST_ASSERT_EQUAL(L3GD20Gyroscope::FIFO_DISABLE, gyro->get_fifo_mode());
    TEST_ASSERT_EQUAL(0, gyro->get_fifo_watermark());
    TEST_ASSERT_EQUAL(L3GD20Gyroscope::HPF_DISABLE, gyro->get_high_pass_filter_mode());
    TEST_ASSERT_EQUAL(L3GD20Gyroscope::HPF_CF0, gyro->get_high_pass_filter_cutoff_freq_mode());
    TEST_ASSERT_EQUAL(L3GD20Gyroscope::LPF_CF0, gyro->get_low_pass_filter_cutoff_freq_mode());
    TEST_ASSERT_EQUAL(L3GD20Gyroscope::ODR_95_HZ, gyro->get_output_data_rate());
    TEST_ASSERT_EQUAL(L3GD20Gyroscope::DRDY_DISABLE, gyro->get_sensitivity_dps());
    TEST_ASSERT_EQUAL(L3GD20Gyroscope::DRDY_DISABLE, gyro->get_data_ready_interrupt_mode());
    TEST_ASSERT_EQUAL(L3GD20Gyroscope::G_DISABLE, gyro->get_gyroscope_mode());
}

void test_multiple_start_stop()
{
    L3GD20Gyroscope::GyroscopeMode mode;
    gyro->set_gyroscope_mode(gyro->G_DISABLE);

    for (int i = 0; i < 4; i++) {
        mode = gyro->get_gyroscope_mode();
        TEST_ASSERT_EQUAL(gyro->G_DISABLE, mode);

        gyro->set_gyroscope_mode(gyro->G_ENABLE);
        mode = gyro->get_gyroscope_mode();
        TEST_ASSERT_EQUAL(gyro->G_ENABLE, mode);

        ThisThread::sleep_for(20ms);

        mode = gyro->get_gyroscope_mode();
        TEST_ASSERT_EQUAL(gyro->G_ENABLE, mode);

        gyro->set_gyroscope_mode(gyro->G_DISABLE);
        mode = gyro->get_gyroscope_mode();
        TEST_ASSERT_EQUAL(gyro->G_DISABLE, mode);

        ThisThread::sleep_for(20ms);
    }
}

float abs_vec3(float vec3[3])
{
    return sqrtf(vec3[0] * vec3[0] + vec3[1] * vec3[1] + vec3[2] * vec3[2]);
}

/**
 * Test gyroscope without moving.
 */
void test_simple_data_reading()
{
    const int n_samples = 8;
    const int dt_ms = 50;
    float gyro_vec[3];
    float angular_velocity_abs[n_samples];
    float angle = 0.0f;

    gyro->set_output_data_rate(L3GD20Gyroscope::ODR_95_HZ);

    for (int i = 0; i < n_samples; i++) {
        gyro->read_data(gyro_vec);
        angular_velocity_abs[i] = abs_vec3(gyro_vec);
        angle += dt_ms * angular_velocity_abs[i] / 1000.0f;
        ThisThread::sleep_for(chrono::milliseconds(dt_ms));
    }

    // check that sensor has noise
    TEST_ASSERT_NOT_EQUAL(0, angle);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, angle);
    for (int i = 1; i < n_samples; i++) {
        TEST_ASSERT_NOT_EQUAL(angular_velocity_abs[i - 1], angular_velocity_abs[i]);
    }
}

struct interrupt_counter_t {
    int samples_count;
    int invokation_count;
    float angle;
    float dt;

    const int samples_per_invokation;

    interrupt_counter_t(int samples_count, int invokation_count, float angle, float dt, int samples_per_invokation)
        : samples_count(samples_count)
        , invokation_count(invokation_count)
        , angle(angle)
        , dt(dt)
        , samples_per_invokation(samples_per_invokation)
    {
    }

    void process_interrupt()
    {
        invokation_count++;
        float ang_velocity[3];

        for (int i = 0; i < samples_per_invokation; i++) {
            samples_count++;

            gyro->read_data(ang_velocity);
            angle += abs_vec3(ang_velocity) * dt;
        }
    }
};

/**
 * Test interrupt usage.
 */
void test_simple_interrupt_usage()
{
    // gyroscope preparation
    gyro->set_output_data_rate(L3GD20Gyroscope::ODR_95_HZ);
    InterruptIn drdy_pin(MBED_CONF_L3GD20_DRIVER_TEST_DRDY);
    interrupt_counter_t interrupt_counter(0, 0, 0, 1.0f / gyro->get_output_data_rate_hz(), 1);
    Event<void()> interrupt_event = mbed_event_queue()->event(callback(&interrupt_counter, &interrupt_counter_t::process_interrupt));
    drdy_pin.rise(callback(&interrupt_event, &Event<void()>::call));

    // run interrupts
    gyro->set_data_ready_interrupt_mode(L3GD20Gyroscope::DRDY_ENABLE);
    // wait processing
    ThisThread::sleep_for(500ms);
    // disable interrupts
    gyro->set_data_ready_interrupt_mode(L3GD20Gyroscope::DRDY_DISABLE);
    drdy_pin.disable_irq();
    ThisThread::sleep_for(100ms);

    // check results
    TEST_ASSERT(interrupt_counter.samples_count > 40);
    TEST_ASSERT(interrupt_counter.samples_count < 60);

    TEST_ASSERT_NOT_EQUAL(0.0f, interrupt_counter.angle);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, interrupt_counter.angle);
}

/**
 * Test FIFO and interrupt usage.
 */
void test_fifo_interrupt_usage()
{
    // gyroscope preparation
    const int fifo_watermark = 24;
    gyro->set_output_data_rate(L3GD20Gyroscope::ODR_95_HZ);
    gyro->set_fifo_watermark(fifo_watermark);
    gyro->set_fifo_mode(L3GD20Gyroscope::FIFO_ENABLE);
    InterruptIn drdy_pin(MBED_CONF_L3GD20_DRIVER_TEST_DRDY);
    interrupt_counter_t interrupt_counter(0, 0, 0, 1.0f / gyro->get_output_data_rate_hz(), fifo_watermark);
    Event<void()> interrupt_event = mbed_event_queue()->event(callback(&interrupt_counter, &interrupt_counter_t::process_interrupt));
    drdy_pin.rise(callback(&interrupt_event, &Event<void()>::call));

    // run interrupts
    gyro->set_data_ready_interrupt_mode(L3GD20Gyroscope::DRDY_ENABLE);
    // wait processing
    ThisThread::sleep_for(1125ms);
    // disable interrupts
    gyro->set_data_ready_interrupt_mode(L3GD20Gyroscope::DRDY_DISABLE);
    drdy_pin.disable_irq();
    ThisThread::sleep_for(100ms);

    // check results
    TEST_ASSERT_EQUAL(4, interrupt_counter.invokation_count);
    TEST_ASSERT_EQUAL(4 * fifo_watermark, interrupt_counter.samples_count);

    TEST_ASSERT_NOT_EQUAL(0.0f, interrupt_counter.angle);
    TEST_ASSERT_FLOAT_WITHIN(0.05f, 0.0f, interrupt_counter.angle);
}

// test cases description
#define GyroCase(test_fun) Case(#test_fun, case_setup_handler, test_fun, greentea_case_teardown_handler, greentea_case_failure_continue_handler)
Case cases[] = {
    GyroCase(test_init_state_enabled),
    GyroCase(test_init_state_disabled),
    GyroCase(test_multiple_start_stop),
    GyroCase(test_simple_data_reading),
    GyroCase(test_simple_interrupt_usage),
    GyroCase(test_fifo_interrupt_usage)
};
Specification specification(test_setup_handler, cases, test_teardown_handler);

// Entry point into the tests
int main()
{
    // host handshake
    // note: should be invoked here or in the test_setup_handler
    GREENTEA_SETUP(40, "default_auto");
    // run tests
    return !Harness::run(specification);
}
