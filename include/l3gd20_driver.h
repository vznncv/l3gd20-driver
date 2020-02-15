#ifndef L3GD20_DRIVER_H
#define L3GD20_DRIVER_H

#include "l3gd20_utils.h"
#include "mbed.h"

namespace l3gd20 {

/**
 * The L3GD20 gyroscope driver.
 */
class L3GD20Gyroscope {
public:
    /**
     * Constructor.
     *
     * @param i2c_ptr I2C interface
     */
    L3GD20Gyroscope(I2C *i2c_ptr);

    /**
     * Constructor.
     *
     * @param sda I2C data line pin
     * @param scl I2C clock line pin
     */
    L3GD20Gyroscope(PinName sda, PinName scl);

    /**
     * Constructor.
     *
     * @param spi_ptr SPI interface
     */
    L3GD20Gyroscope(SPI *spi_ptr, PinName ssel);

    /**
     * Constructor.
     *
     * @param mosi SPI mosi pin
     * @param miso SPI miso pin
     * @param sclk SPI sclk pin
     * @param ssel SPI ssel pin
     */
    L3GD20Gyroscope(PinName mosi, PinName miso, PinName sclk, PinName ssel);

    virtual ~L3GD20Gyroscope();

    /**
     * Initialize device with default settings and test connection.
     *
     * Note: this method is idempotent.
     *
     * @param start if it's `true`, then initially sensor will be enabled, otherwise disabled.
     * @return 0, if device is initialize correctly, otherwise non-zero error code.
     */
    int init(bool start = true);

    enum Registers {
        WHO_AM_I_ADDR = 0x0F, // device identification register
        CTRL_REG1_ADDR = 0x20, // Control register 1
        CTRL_REG2_ADDR = 0x21, // Control register 2
        CTRL_REG3_ADDR = 0x22, // Control register 3
        CTRL_REG4_ADDR = 0x23, // Control register 4
        CTRL_REG5_ADDR = 0x24, // Control register 5
        REFERENCE_REG_ADDR = 0x25, // Reference register
        OUT_TEMP_ADDR = 0x26, // Out temp register
        STATUS_REG_ADDR = 0x27, // Status register */
        OUT_X_L_ADDR = 0x28, // Output Register X
        OUT_X_H_ADDR = 0x29, // Output Register X
        OUT_Y_L_ADDR = 0x2A, // Output Register Y
        OUT_Y_H_ADDR = 0x2B, // Output Register Y
        OUT_Z_L_ADDR = 0x2C, // Output Register Z
        OUT_Z_H_ADDR = 0x2D, // Output Register Z
        FIFO_CTRL_REG_ADDR = 0x2E, // Fifo control Register
        FIFO_SRC_REG_ADDR = 0x2F, // Fifo src Register
        INT1_CFG_ADDR = 0x30, // Interrupt 1 configuration Register
        INT1_SRC_ADDR = 0x31, // Interrupt 1 source Register
        INT1_TSH_XH_ADDR = 0x32, // Interrupt 1 Threshold X register
        INT1_TSH_XL_ADDR = 0x33, // Interrupt 1 Threshold X register
        INT1_TSH_YH_ADDR = 0x34, // Interrupt 1 Threshold Y register
        INT1_TSH_YL_ADDR = 0x35, // Interrupt 1 Threshold Y register
        INT1_TSH_ZH_ADDR = 0x36, // Interrupt 1 Threshold Z register
        INT1_TSH_ZL_ADDR = 0x37, // Interrupt 1 Threshold Z register
        INT1_DURATION_ADDR = 0x38, // Interrupt 1 DURATION register
    };

    /**
     * Read register.
     *
     * @param reg register address
     * @return register value
     */
    uint8_t read_register(uint8_t reg);

    /**
     * Write value to register.
     *
     * @param reg register address
     * @param val register value
     */
    void write_register(uint8_t reg, uint8_t val);

    enum GyroscopeMode {
        G_DISABLE = 0x00,
        G_ENABLE = 0x0F
    };
    /**
     * Enable/disable gyroscope.
     *
     * @param mode
     */
    void set_gyroscope_mode(GyroscopeMode mode);

    /**
     * Check if gyroscope is enabled/disabled.
     *
     * @return 0 if gyroscope is disabled, otherwise non-zero value
     */
    GyroscopeMode get_gyroscope_mode();

    enum OutputDataRate {
        ODR_95_HZ = 0x00,
        ODR_190_HZ = 0x40,
        ODR_380_HZ = 0x80,
        ODR_760_HZ = 0xC0
    };

    /**
     * Set output data rate.
     *
     * @param odr
     */
    void set_output_data_rate(OutputDataRate odr);

    /**
     * Get output data rate.
     *
     * @return
     */
    OutputDataRate get_output_data_rate();

    /**
     * Get output data rate in HZ.
     *
     * @return
     */
    float get_output_data_rate_hz();

    enum LowPassFilterCutoffFreqMode {
        LPF_CF0 = 0x00,
        LPF_CF1 = 0x10,
        LPF_CF2 = 0x20,
        LPF_CF3 = 0x30
    };

    /**
     * Set low pass filter cutoff frequency mode.
     *
     * The low pass filter cutoff frequency depends on filter mode and output data rate.
     * For more details see L3GD20 datasheet.
     *
     * @param mode
     */
    void set_low_pass_filter_cutoff_freq_mode(LowPassFilterCutoffFreqMode mode);

    /**
     * Get low pass filter cutoff frequency mode.
     *
     * @return
     */
    LowPassFilterCutoffFreqMode get_low_pass_filter_cutoff_freq_mode();

    /**
     * Calculate current cutoff frequency of the low pass filter in HZ.
     *
     * @return
     */
    float get_low_pass_filter_cut_off_frequency();

    enum HighPassFilterMode {
        HPF_ENABLE = 0x10,
        HPF_DISABLE = 0x00
    };

    /**
     * Disable/enable high pass filter.
     *
     * @param mode
     */
    void set_high_pass_filter_mode(HighPassFilterMode mode);

    /**
     * Check if high pass filter is enabled/disabled.
     *
     * @return 0 if gyroscope is disabled, otherwise non-zero value
     */
    HighPassFilterMode get_high_pass_filter_mode();

    enum HighPassFilterCutoffFreqMode {
        HPF_CF0 = 0x00,
        HPF_CF1 = 0x01,
        HPF_CF2 = 0x02,
        HPF_CF3 = 0x03,
        HPF_CF4 = 0x04,
        HPF_CF5 = 0x05,
        HPF_CF6 = 0x06,
        HPF_CF7 = 0x07,
        HPF_CF8 = 0x08,
        HPF_CF9 = 0x09
    };

    /**
     * Set high pass filter cutoff frequency mode.
     *
     * The high pass filter cutoff frequency depends on filter mode and output data rate.
     * For more details see L3GD20 datasheet.
     *
     * @param mode
     */
    void set_high_pass_filter_cutoff_freq_mode(HighPassFilterCutoffFreqMode mode);

    /**
     * Get high pass filter cutoff frequency mode.
     *
     * @return
     */
    HighPassFilterCutoffFreqMode get_high_pass_filter_cutoff_freq_mode();

    /**
     * Calculate current cutoff frequency of the high pass filter in HZ.
     *
     * @return
     */
    float get_high_pass_filter_cut_off_frequency();

    enum FullScale {
        FULL_SCALE_250 = 0x00,
        FULL_SCALE_500 = 0x10,
        FULL_SCALE_1000 = 0x20,
        FULL_SCALE_2000 = 0x30
    };

    /**
     * Get full scale, i.e. maximum degrees per second (dps) value that gyroscope can detect.
     *
     * @note
     * Full scale influences on sensitivity.
     *
     * @param fs
     */
    void set_full_scale(FullScale fs);

    /**
     * Get current full scale.
     *
     * @return
     */
    FullScale get_full_scale();

    /**
     * Get sensor sensitivity in radian per seconds per LSB (rad/(s*LSB)).
     *
     * @return
     */
    float get_sensitivity();

    /**
     * Get sensor sensitivity in degrees per second per LSB (dps/LSB).
     *
     * @return
     */
    float get_sensitivity_dps();

    enum FIFOMode {
        FIFO_ENABLE = 1,
        FIFO_DISABLE = 0
    };

    /**
     * Enable/disabled FIFO.
     *
     * @param mode
     */
    void set_fifo_mode(FIFOMode mode);

    /**
     * Check if FIFO is enabled/disabled.
     *
     * @return 0 if FIFO is disabled, otherwise non-zero value
     */
    FIFOMode get_fifo_mode();

    /**
     * Set FIFO watermark.
     *
     * @param watermark value between 0 an 31.
     */
    void set_fifo_watermark(int watermark);

    /**
     * Get current FIFO watermark value.
     *
     * @return watermark value
     */
    int get_fifo_watermark();

    /**
     * Clear FIFO content.
     */
    void clear_fifo();

    enum DataReadyInterruptMode {
        DRDY_ENABLE = 1,
        DRDY_DISABLE = 0
    };

    /**
     * Enable/disable data ready interrupt on pin INT1.
     *
     * If FIFO is enabled, interrupt will be configured for FIFO watermark.
     *
     * @param drdy_mode
     */
    void set_data_ready_interrupt_mode(DataReadyInterruptMode drdy_mode);

    /**
     * Check if data ready interrupt is disabled/enabled.
     *
     * @return 0 if interrupt is disabled, otherwise non-zero value
     */
    DataReadyInterruptMode get_data_ready_interrupt_mode();

    /**
     * Read current gyroscope data.
     *
     * The data will be placed into \p data array in order: x, y, z.
     * The values will be converted into radians per seconds (rad/s).
     *
     * @param data
     */
    void read_data(float data[3]);

    /**
     * Read current gyroscope data.
     *
     * The data will be placed into \p data array in order: x, y, z.
     * The values will be converted into degrees per seconds (dps).
     *
     * @param data
     */
    void read_data_dps(float data[3]);

    /**
     * Read raw gyroscope data.
     *
     * The data will be placed into \p data array in order: x, y, z.
     * The values represent signed integers.
     *
     * To convert raw data into rad/s or dps, it should be multiplied by value
     * that is returned by get_sensitivity() or get_sensitivity_dps() correspondingly.
     *
     * @param data
     */
    void read_data_16(int16_t data[3]);

    /**
     * Get raw data from temperature sensor.
     *
     * @note
     * Zero level of the temperature sensor isn't calibrated!
     *
     * To get relative temperature in the Celsius, use the formula: t_c = val * sensitivity.
     *
     * @return
     */
    int8_t read_temperature_8();

    /**
     * Get temperature sensor sensitivity.
     *
     * @return
     */
    float get_temperature_sensor_sensitivity();

private:
    RegisterDevice _register_device;

    static const uint8_t _DEVICE_ID = 0xD4;

    /**
     * Update interrupt register.
     *
     * \p mode values:
     * - 0 - disable
     * - 1 - enable
     * - 2 - update configuration
     * - 3 - check status
     *
     * @param mode
     */
    DataReadyInterruptMode _update_interrupt_register(int mode);

    // current gyroscope sensitivity
    float _gyro_sensitivity_dps;
    float _gyro_sensitivity_rps;
};
}

using l3gd20::L3GD20Gyroscope;

#endif // L3GD20_DRIVER_H
