/**
 * Example of the L3GD20 usage with STM32F3Discovery board.
 *
 * Interrupt and FIFO usage.
 *
 * This sample integrates data, using quaternion math to show current rotation.
 * See: http://stanford.edu/class/ee267/lectures/lecture10.pdf for more details
 *
 * Pin map:
 *
 * - PC_4 - UART TX (stdout/stderr)
 * - PC_5 - UART RX (stdin)
 * - PA_7 - SPI MOSI of the L3GD20
 * - PA_6 - SPI MISO of the L3GD20
 * - PA_5 - SPI SCLK of the L3GD20
 * - PE_3 - SPI SSEL of the L3GD20
 * - PE_1 - INT2 pin of the L3GD20
 */
#include "l3gd20_driver.h"
#include "math.h"
#include "mbed.h"

class GyroProcessor {
public:
    GyroProcessor(L3GD20Gyroscope *gyro, int block_size, PinName drdy_pin, PinName indicator)
        : _gyro(gyro)
        , _drdy_int(drdy_pin)
        , _indicator_out(indicator)
        , _block_size(block_size)
        , _sensor_queue()
        , _sensor_thread(osPriorityHigh7)
    {
        // configure gyroscope
        _drdy_int.disable_irq();

        _w_offset[0] = 0.0f;
        _w_offset[1] = 0.0f;
        _w_offset[2] = 0.0f;
    }

    /**
     * Calibrate gyroscope to eliminate offset error.
     *
     * During calibration it shouldn't be moved.
     *
     * @param calibration_time
     */
    void calibrate(float calibration_time)
    {
        _dt = 1.0f / _gyro->get_output_data_rate_hz();
        _calibration_samples_count = 0;
        _w_offset[0] = 0.0f;
        _w_offset[1] = 0.0f;
        _w_offset[2] = 0.0f;
        _drdy_int.rise(_sensor_queue.event(callback(this, &GyroProcessor::_calibrate_callback)));

        _gyro->set_fifo_watermark(_block_size);
        _gyro->clear_fifo();
        _gyro->set_fifo_mode(L3GD20Gyroscope::FIFO_ENABLE);
        _gyro->set_data_ready_interrupt_mode(L3GD20Gyroscope::DRDY_ENABLE);
        _drdy_int.enable_irq();
        _sensor_queue.dispatch(calibration_time * 1000);
        _drdy_int.disable_irq();
        _gyro->set_data_ready_interrupt_mode(L3GD20Gyroscope::DRDY_DISABLE);

        if (_calibration_samples_count) {
            for (int i = 0; i < 3; i++) {
                _w_offset[i] /= _calibration_samples_count;
                _w_offset[i] = -_w_offset[i];
            }
        }
    }

    void start_async()
    {
        _dt = 1.0f / _gyro->get_output_data_rate_hz();
        _gyro->set_fifo_watermark(_block_size);
        _gyro->clear_fifo();
        _gyro->set_fifo_mode(L3GD20Gyroscope::FIFO_ENABLE);
        _gyro->set_data_ready_interrupt_mode(L3GD20Gyroscope::DRDY_ENABLE);
        _drdy_int.enable_irq();

        // start quaternion
        q[0] = 1.0f;
        q[1] = 0.0f;
        q[2] = 0.0f;
        q[3] = 0.0f;

        // run processing thread
        _drdy_int.rise(_sensor_queue.event(callback(this, &GyroProcessor::_process_block)));
        _sensor_thread.start(callback(&_sensor_queue, &EventQueue::dispatch_forever));
    }

    /**
     * Get current object rotation.
     *
     * @param angle
     * @param vec
     */
    void get_rotation(float *angle_ptr, float vec[3])
    {
        float current_q[4];
        _mutex.lock();
        memcpy(current_q, q, sizeof(float) * 4);
        _mutex.unlock();

        _quaternion_to_roration(current_q, angle_ptr, vec);
    }

private:
    L3GD20Gyroscope *_gyro;
    InterruptIn _drdy_int;
    DigitalOut _indicator_out;

    int _block_size;
    float _dt;
    Mutex _mutex;

    EventQueue _sensor_queue;
    Thread _sensor_thread;

    // quaternion that describe current rotation
    float q[4];

    // calibration constains
    float _w_offset[3];
    int _calibration_samples_count;

    void _calibrate_callback()
    {
        // disable drdy irq to prevent accident interrupt during FIFO reading
        _drdy_int.disable_irq();
        float w[3];

        for (int i = 0; i < _block_size; i++) {
            // read data
            _gyro->read_data(w);

            // accumulate offset
            _w_offset[0] += w[0];
            _w_offset[1] += w[1];
            _w_offset[2] += w[2];

            _calibration_samples_count++;
        }
        _drdy_int.enable_irq();
    }

    void _process_block()
    {
        // disable drdy irq to prevent accident interrupt during FIFO reading
        _drdy_int.disable_irq();
        _indicator_out = !_indicator_out;
        float w[3];
        float angle;
        float delta_q[4];
        float current_q[4];
        memcpy(current_q, q, sizeof(float) * 4);

        for (int i = 0; i < _block_size; i++) {
            // read data
            _gyro->read_data(w);

            // compensate offset
            w[0] += _w_offset[0];
            w[1] += _w_offset[1];
            w[2] += _w_offset[2];

            // get rotation quaternion from current accelerometer data
            // notes:
            //  - we assumes, that (w[0] * dt, w[1] * dt, w[2] * dt)
            //    represents Euler angles and using simplified formula,
            //    convert them into quaternion form
            //  - due small angles, the Euler angles rotation order isn't important
            angle = sqrtf(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]) * _dt;
            _roration_to_quaternion(angle, w, delta_q);
            // integrate
            _prodution_quaterniona(current_q, delta_q, current_q);
            // normalize quaternion
            _normalize_quaternion(current_q);
        }
        _indicator_out = !_indicator_out;
        _drdy_int.enable_irq();

        // update quaternion value
        _mutex.lock();
        memcpy(q, current_q, sizeof(float) * 4);
        _mutex.unlock();
    }

    /**
     * Calculate quaternion production.
     *
     * out = p * q
     *
     * @param p
     * @param q
     * @param out
     */
    void _prodution_quaterniona(const float p[4], const float q[4], float out[4])
    {
        float ow = p[0] * q[0] - p[1] * q[1] - p[2] * q[2] - p[3] * q[3];
        float ox = p[0] * q[1] + p[1] * q[0] + p[2] * q[3] - p[3] * q[2];
        float oy = p[0] * q[2] - p[1] * q[3] + p[2] * q[0] + p[3] * q[1];
        float oz = p[0] * q[3] + p[1] * q[2] - p[2] * q[1] + p[3] * q[0];
        out[0] = ow;
        out[1] = ox;
        out[2] = oy;
        out[3] = oz;
    }

    /**
     * Normalize quaternion.
     *
     * @param p
     */
    void _normalize_quaternion(float q[4])
    {
        float k = 1.0f / sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        q[0] *= k;
        q[1] *= k;
        q[2] *= k;
        q[3] *= k;
    }

    /**
     * Convert quaternion to rotation axis and angle.
     *
     * @param q
     * @param angle_ptr
     * @param r
     */
    void _quaternion_to_roration(const float q[4], float *angle_ptr, float r[3])
    {
        float half_angle = acosf(q[0]);
        float r_a;
        float norm_k = 1.0f / sqrtf(q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        for (int i = 0; i < 3; i++) {
            r_a = norm_k * q[i + 1];
            if (!(r_a < 1)) {
                r_a = 1;
            }
            if (!(r_a > -1)) {
                r_a = -1;
            }
            r[i] = r_a;
        }

        *angle_ptr = 2 * half_angle;
    }
    /**
     * Convert rotation to quaternion.
     *
     * @param angle
     * @param r
     * @param q
     */
    void _roration_to_quaternion(const float angle, const float r[3], float q[4])
    {
        float half_angle = angle / 2;
        float norm_k = 1.0f / sqrtf(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
        float norm_r[3];
        for (int i = 0; i < 3; i++) {
            float r_a = r[i] * norm_k;
            if (!(r_a > -1)) {
                r_a = -1;
            }
            if (!(r_a < 1)) {
                r_a = 1;
            }
            norm_r[i] = r_a;
        }
        // calculate quaternion
        float sin_half_angle = sinf(half_angle);
        q[0] = cosf(half_angle);
        q[1] = sin_half_angle * norm_r[0];
        q[2] = sin_half_angle * norm_r[1];
        q[3] = sin_half_angle * norm_r[2];
    }
};

DigitalOut led(LED2);

EventQueue sensor_queue;
Thread sensor_thread(osPriorityHigh7);

int main()
{
    // create separate spi instance
    SPI spi(PA_7, PA_6, PA_5);
    spi.frequency(10000000);
    L3GD20Gyroscope gyroscope(&spi, PE_3);
    // initialize device
    int err = gyroscope.init();
    if (err) {
        MBED_ERROR(MBED_ERROR_INITIALIZATION_FAILED, "Gyroscope initialization failed");
    }

    // set gyroscope parameter explicitly
    gyroscope.set_output_data_rate(L3GD20Gyroscope::ODR_760_HZ);
    gyroscope.set_full_scale(L3GD20Gyroscope::FULL_SCALE_250);
    gyroscope.set_low_pass_filter_cutoff_freq_mode(L3GD20Gyroscope::LPF_CF0);
    //gyroscope.set_high_pass_filter_cutoff_freq_mode(L3GD20Gyroscope::HPF_CF9);
    gyroscope.set_high_pass_filter_mode(L3GD20Gyroscope::HPF_DISABLE);

    // create helper object to read and process gyroscope data
    int block_size = 24;
    GyroProcessor gyro_processor(&gyroscope, block_size, PE_1, LED5);
    // run calibration
    wait(0.1f);
    gyro_processor.calibrate(0.9f);
    // run data processing
    gyro_processor.start_async();

    // rotation data
    float angle;
    float rotation_vec[3];

    while (true) {
        led = !led;
        gyro_processor.get_rotation(&angle, rotation_vec);
        printf("angle: %+6.2f; x: %+6.2f; y: %+6.2f; z: %+6.2f\n", angle, rotation_vec[0], rotation_vec[1], rotation_vec[2]);
        wait(0.016f);
        led = !led;
        wait(0.016f);
    }
}
