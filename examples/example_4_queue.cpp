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
    GyroProcessor(L3GD20Gyroscope* gyro, int block_size, PinName drdy_pin, PinName indicator)
        : gyro(gyro)
        , drdy_int(drdy_pin)
        , indicator_out(indicator)
        , block_size(block_size)
    {
        // configure gyroscope
        drdy_int.disable_irq();
    }

    void start()
    {
        dt = 1.0f / gyro->get_output_data_rate_hz();
        gyro->set_fifo_watermark(block_size);
        gyro->clear_fifo();
        gyro->set_fifo_mode(L3GD20Gyroscope::FIFO_ENABLE);
        gyro->set_data_ready_interrupt_mode(L3GD20Gyroscope::DRDY_ENABLE);
        drdy_int.enable_irq();

        // start quaternion
        q[0] = 1.0f;
        q[1] = 0.0f;
        q[2] = 0.0f;
        q[3] = 0.0f;
    }

    void add_data_processing_event(EventQueue* sensor_processing_queue)
    {
        drdy_int.rise(sensor_processing_queue->event(callback(this, &GyroProcessor::process_block)));
    }

    void process_block()
    {
        // disable drdy irq to prevent accident interrupt during FIFO reading
        drdy_int.disable_irq();
        indicator_out = !indicator_out;
        float w[3];
        float angle;
        float delta_q[4];
        float current_q[4];
        memcpy(current_q, q, sizeof(float) * 4);

        for (int i = 0; i < block_size; i++) {
            // read data
            gyro->read_data(w);
            // convert degrees per second to radian
            w[0] *= DEG_TO_RADIAN;
            w[1] *= DEG_TO_RADIAN;
            w[2] *= DEG_TO_RADIAN;
            // get rotation quaternion from current accelerometer data
            // notes:
            //  - we assumes, that (w[0] * dt, w[1] * dt, w[2] * dt)
            //    represents Euler angles and using simplified formula,
            //    convert them into quaternion form
            //  - due small angles, the Euler angles rotation order isn't important
            angle = sqrtf(w[0] * w[0] + w[1] * w[1] + w[2] * w[2]) * dt;
            roration_to_quaternion(angle, w, delta_q);
            // integrate
            prodution_quaterniona(current_q, delta_q, current_q);
            // normalize quaternion
            normalize_quaternion(current_q);
        }
        indicator_out = !indicator_out;
        drdy_int.enable_irq();

        // update quaternion value
        mutex.lock();
        memcpy(q, current_q, sizeof(float) * 4);
        mutex.unlock();
    }
    /**
     * Get current object rotation.
     *
     * @param angle
     * @param vec
     */
    void get_rotation(float* angle_ptr, float vec[3])
    {
        float current_q[4];
        mutex.lock();
        memcpy(current_q, q, sizeof(float) * 4);
        mutex.unlock();

        quaternion_to_roration(current_q, angle_ptr, vec);
    }

private:
    L3GD20Gyroscope* gyro;
    InterruptIn drdy_int;
    DigitalOut indicator_out;

    int block_size;
    float dt;
    Mutex mutex;

    // quaternion that describe current rotation
    float q[4];
    // helper constants
    const float DEG_TO_RADIAN = 0.01745329f;
    // helper function
    /**
     * Calculate quaternion production.
     *
     * out = p * q
     *
     * @param p
     * @param q
     * @param out
     */
    void prodution_quaterniona(const float p[4], const float q[4], float out[4])
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
    void normalize_quaternion(float q[4])
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
    void quaternion_to_roration(const float q[4], float* angle_ptr, float r[3])
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
    void roration_to_quaternion(const float angle, const float r[3], float q[4])
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
    //gyroscope.set_high_pass_filter_mode(L3GD20Gyroscope::HPF_ENABLE);

    // create helper object to read and process gyroscope data
    int block_size = 24;
    GyroProcessor gyro_processor(&gyroscope, block_size, PE_1, LED5);
    gyro_processor.add_data_processing_event(&sensor_queue);
    // run sensor queue
    sensor_thread.start(callback(&sensor_queue, &EventQueue::dispatch_forever));
    // start sensor data processing
    gyro_processor.start();

    // rotation data
    float angle;
    float rotation_vec[3];

    while (true) {
        led = !led;
        gyro_processor.get_rotation(&angle, rotation_vec);
        printf("angle: %+6.2f; x: %+6.2f; y: %+6.2f; z: %+6.2f\n",
            angle, rotation_vec[0], rotation_vec[1], rotation_vec[2]);
        wait(0.005f);
        led = !led;
        wait(0.010f);
    }
}
