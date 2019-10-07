#include "l3gd20_driver.h"

using namespace l3gd20;

L3GD20Gyroscope::L3GD20Gyroscope(I2C *i2c_ptr)
    : register_device(i2c_ptr)
{
}

L3GD20Gyroscope::L3GD20Gyroscope(PinName sda, PinName scl)
    : register_device(sda, scl)
{
}

L3GD20Gyroscope::L3GD20Gyroscope(SPI *spi_ptr, PinName ssel)
    : register_device(spi_ptr, ssel)
{
}

L3GD20Gyroscope::L3GD20Gyroscope(PinName mosi, PinName miso, PinName sclk, PinName ssel)
    : register_device(mosi, miso, sclk, ssel)
{
}

L3GD20Gyroscope::~L3GD20Gyroscope()
{
}

int L3GD20Gyroscope::init(bool start)
{
    // check device id
    uint8_t device_id;

    // sometimes device glitches and returns wrong device_id. So try to extract it several times
    for (int i = 0; i < 3; i++) {
        device_id = register_device.read_register(WHO_AM_I_ADDR);
        if (device_id == DEVICE_ID) {
            break;
        }
    }
    if (device_id != DEVICE_ID) {
        return MBED_ERROR_CODE_INITIALIZATION_FAILED;
    }

    // continuous data update and little endian data order
    register_device.update_register(CTRL_REG4_ADDR, 0x00, 0xC0);
    // connect output to LPF2
    register_device.update_register(CTRL_REG5_ADDR, 0x03, 0x03);

    // default settings
    set_data_ready_interrupt_mode(DRDY_DISABLE);
    set_fifo_mode(FIFO_DISABLE);
    set_fifo_watermark(0);
    set_full_scale(FULL_SCALE_250);
    set_high_pass_filter_mode(HPF_DISABLE);
    set_high_pass_filter_cutoff_freq_mode(HPF_CF0);
    set_low_pass_filter_cutoff_freq_mode(LPF_CF0);
    set_output_data_rate(ODR_95_HZ);
    set_gyroscope_mode(start ? G_ENABLE : G_DISABLE);

    return MBED_SUCCESS;
}

uint8_t L3GD20Gyroscope::read_register(uint8_t reg)
{
    return register_device.read_register(reg);
}

void L3GD20Gyroscope::write_register(uint8_t reg, uint8_t val)
{
    register_device.write_register(reg, val);
}

void L3GD20Gyroscope::set_gyroscope_mode(GyroscopeMode mode)
{
    register_device.update_register(CTRL_REG1_ADDR, mode, 0x0F);
}

L3GD20Gyroscope::GyroscopeMode L3GD20Gyroscope::get_gyroscope_mode()
{
    if (register_device.read_register(CTRL_REG1_ADDR, 0x08)) {
        return G_ENABLE;
    } else {
        return G_DISABLE;
    }
}

void L3GD20Gyroscope::set_output_data_rate(OutputDataRate odr)
{
    register_device.update_register(CTRL_REG1_ADDR, odr, 0xC0);
}

static const L3GD20Gyroscope::OutputDataRate ODR_MODE_MAP[] = {
    L3GD20Gyroscope::ODR_95_HZ,
    L3GD20Gyroscope::ODR_190_HZ,
    L3GD20Gyroscope::ODR_380_HZ,
    L3GD20Gyroscope::ODR_760_HZ,
};

L3GD20Gyroscope::OutputDataRate L3GD20Gyroscope::get_output_data_rate()
{
    uint8_t i = register_device.read_register(CTRL_REG1_ADDR, 0xC0) >> 6;
    return ODR_MODE_MAP[i];
}

static const float ODR_FREQ_MAP[] = {
    95.0f,
    190.0f,
    380.0f,
    760.0f,
};

float L3GD20Gyroscope::get_output_data_rate_hz()
{
    uint8_t val = register_device.read_register(CTRL_REG1_ADDR, 0xC0) >> 6;
    return ODR_FREQ_MAP[val];
}

void L3GD20Gyroscope::set_low_pass_filter_cutoff_freq_mode(LowPassFilterCutoffFreqMode mode)
{
    register_device.update_register(CTRL_REG1_ADDR, mode, 0x30);
}

static const L3GD20Gyroscope::LowPassFilterCutoffFreqMode LPF_CF_MODE_MAP[] = {
    L3GD20Gyroscope::LPF_CF0,
    L3GD20Gyroscope::LPF_CF0,
    L3GD20Gyroscope::LPF_CF0,
    L3GD20Gyroscope::LPF_CF0,
};

L3GD20Gyroscope::LowPassFilterCutoffFreqMode L3GD20Gyroscope::get_low_pass_filter_cutoff_freq_mode()
{
    uint8_t i = register_device.read_register(CTRL_REG1_ADDR, 0x30) >> 4;
    return LPF_CF_MODE_MAP[i];
}

static const float LPF_CF_FREQ_MAP[] = {
    // odr 95 Hz
    15.5f,
    25.0f,
    25.0f,
    25.0f,
    // odr 190 Hz
    12.5f,
    25.0f,
    50.0f,
    70.0f,
    // odr 380 Hz
    20.0f,
    25.0f,
    50.0f,
    100.0f,
    // odr 760 Hz
    30.0f,
    35.0f,
    50.0f,
    100.0f
};

float L3GD20Gyroscope::get_low_pass_filter_cut_off_frequency()
{
    uint8_t i = register_device.read_register(CTRL_REG1_ADDR, 0xF0) >> 4;
    return LPF_CF_FREQ_MAP[i];
}

void L3GD20Gyroscope::set_high_pass_filter_mode(HighPassFilterMode mode)
{
    register_device.update_register(CTRL_REG5_ADDR, mode, 0x10);
}

L3GD20Gyroscope::HighPassFilterMode L3GD20Gyroscope::get_high_pass_filter_mode()
{
    if (register_device.read_register(CTRL_REG5_ADDR, 0x10)) {
        return HPF_ENABLE;
    } else {
        return HPF_DISABLE;
    }
}

void L3GD20Gyroscope::set_high_pass_filter_cutoff_freq_mode(HighPassFilterCutoffFreqMode mode)
{
    register_device.update_register(CTRL_REG2_ADDR, mode, 0x0F);
}

static const L3GD20Gyroscope::HighPassFilterCutoffFreqMode HPF_CF_MODE_MAP[] = {
    L3GD20Gyroscope::HPF_CF0,
    L3GD20Gyroscope::HPF_CF1,
    L3GD20Gyroscope::HPF_CF2,
    L3GD20Gyroscope::HPF_CF3,
    L3GD20Gyroscope::HPF_CF4,
    L3GD20Gyroscope::HPF_CF5,
    L3GD20Gyroscope::HPF_CF6,
    L3GD20Gyroscope::HPF_CF7,
    L3GD20Gyroscope::HPF_CF8,
    L3GD20Gyroscope::HPF_CF9,
};

L3GD20Gyroscope::HighPassFilterCutoffFreqMode L3GD20Gyroscope::get_high_pass_filter_cutoff_freq_mode()
{
    uint8_t i = register_device.read_register(CTRL_REG2_ADDR, 0x0F);
    if (i >= 10) {
        i = 9;
    }
    return HPF_CF_MODE_MAP[i];
}

static const float HPF_CF_FREQ_MAP[] = {
    // mode 0
    7.2f,
    13.5f,
    27.0f,
    51.4f,
    // mode 1
    3.5f,
    7.2f,
    13.5f,
    27.0f,
    // mode 2
    1.8f,
    3.5f,
    7.2f,
    13.5f,
    // mode 3
    0.9f,
    1.8f,
    3.5f,
    7.2f,
    // mode 4
    0.45f,
    0.9f,
    1.8f,
    3.5f,
    // mode 5
    0.18f,
    0.45f,
    0.9f,
    1.8f,
    // mode 6
    0.09f,
    0.18f,
    0.45f,
    0.9f,
    // mode 7
    0.045f,
    0.09f,
    0.18f,
    0.45f,
    // mode 8
    0.018f,
    0.045f,
    0.09f,
    0.18f,
    // mode 9
    0.009f,
    0.018f,
    0.045f,
    0.09f,
};

float L3GD20Gyroscope::get_high_pass_filter_cut_off_frequency()
{
    uint8_t hfp_cf_mode = register_device.read_register(CTRL_REG2_ADDR, 0x0F);
    if (hfp_cf_mode >= 10) {
        hfp_cf_mode = 9;
    }
    uint8_t odr_mode = register_device.read_register(CTRL_REG1_ADDR, 0xC0) >> 6;
    uint8_t i = (uint8_t)(hfp_cf_mode << 2) | odr_mode;
    return HPF_CF_FREQ_MAP[i];
}

static const L3GD20Gyroscope::FullScale FS_MODE_MAP[] = {
    L3GD20Gyroscope::FULL_SCALE_250,
    L3GD20Gyroscope::FULL_SCALE_500,
    L3GD20Gyroscope::FULL_SCALE_1000,
    L3GD20Gyroscope::FULL_SCALE_2000,
};

static const float SENSITIVITY_MAP[] = {
    0.00875f,
    0.01750f,
    0.03500f,
    0.07000f,
};

static const float RADIAN_PER_DEGREE = 0.017453292519943295f;

void L3GD20Gyroscope::set_full_scale(FullScale fs)
{
    register_device.update_register(CTRL_REG4_ADDR, fs, 0x30);
    uint8_t i = (fs & 0x30) >> 4;
    gyro_sensitivity_dps = SENSITIVITY_MAP[i];
    gyro_sensitivity_rps = gyro_sensitivity_dps * RADIAN_PER_DEGREE;
}

L3GD20Gyroscope::FullScale L3GD20Gyroscope::get_full_scale()
{
    uint8_t i = register_device.read_register(CTRL_REG4_ADDR, 0x30) >> 4;
    return FS_MODE_MAP[i];
}

float L3GD20Gyroscope::get_sensitivity()
{
    uint8_t i = register_device.read_register(CTRL_REG4_ADDR, 0x30) >> 4;
    return SENSITIVITY_MAP[i] * RADIAN_PER_DEGREE;
}

float L3GD20Gyroscope::get_sensitivity_dps()
{
    uint8_t i = register_device.read_register(CTRL_REG4_ADDR, 0x30) >> 4;
    return SENSITIVITY_MAP[i];
}

void L3GD20Gyroscope::set_fifo_mode(FIFOMode mode)
{
    if (mode) {
        register_device.update_register(FIFO_CTRL_REG_ADDR, 0x40, 0xE0); // configure FIFO stream mode
        register_device.update_register(CTRL_REG5_ADDR, 0x40, 0x40); // enable FIFO
    } else {
        register_device.update_register(CTRL_REG5_ADDR, 0x00, 0x40); // disabled FIFO
        register_device.update_register(FIFO_CTRL_REG_ADDR, 0x00, 0xE0); // configure FIFO bypass mode
    }
    _update_interrupt_register(2);
}

L3GD20Gyroscope::FIFOMode L3GD20Gyroscope::get_fifo_mode()
{
    if (register_device.read_register(CTRL_REG5_ADDR, 0x40)) {
        return FIFO_ENABLE;
    } else {
        return FIFO_DISABLE;
    }
}

void L3GD20Gyroscope::set_fifo_watermark(int watermark)
{
    if (watermark < 0 || watermark >= 32) {
        MBED_ERROR(MBED_ERROR_INVALID_ARGUMENT, "Invalid watermark value");
    }
    register_device.update_register(FIFO_CTRL_REG_ADDR, (uint8_t)watermark, 0x1F);
}

int L3GD20Gyroscope::get_fifo_watermark()
{
    return register_device.read_register(FIFO_CTRL_REG_ADDR, 0x1F);
}

void L3GD20Gyroscope::clear_fifo()
{
    uint8_t fifo_mode = register_device.read_register(FIFO_CTRL_REG_ADDR, 0xC0);
    if (fifo_mode != 0) {
        // switch to bypass mode and back
        // (this action will clear FIFO)
        register_device.update_register(FIFO_CTRL_REG_ADDR, 0x00, 0xC0);
        register_device.update_register(FIFO_CTRL_REG_ADDR, fifo_mode, 0xC0);
    }
}

void L3GD20Gyroscope::set_data_ready_interrupt_mode(DataReadyInterruptMode drdy_mode)
{
    if (drdy_mode) {
        _update_interrupt_register(1);
    } else {
        _update_interrupt_register(0);
    }
}

L3GD20Gyroscope::DataReadyInterruptMode L3GD20Gyroscope::get_data_ready_interrupt_mode()
{
    return _update_interrupt_register(3);
}

void L3GD20Gyroscope::read_data(float data[3])
{
    int16_t data_16[3];
    read_data_16(data_16);
    data[0] = data_16[0] * gyro_sensitivity_rps;
    data[1] = data_16[1] * gyro_sensitivity_rps;
    data[2] = data_16[2] * gyro_sensitivity_rps;
}

void L3GD20Gyroscope::read_data_dps(float data[3])
{
    int16_t data_16[3];
    read_data_16(data_16);
    data[0] = data_16[0] * gyro_sensitivity_dps;
    data[1] = data_16[1] * gyro_sensitivity_dps;
    data[2] = data_16[2] * gyro_sensitivity_dps;
}

void L3GD20Gyroscope::read_data_16(int16_t data[3])
{
    uint8_t raw_data[6];
    register_device.read_registers(OUT_X_L_ADDR, raw_data, 6);
    data[0] = (int16_t)(raw_data[1] << 8) + raw_data[0];
    data[1] = (int16_t)(raw_data[3] << 8) + raw_data[2];
    data[2] = (int16_t)(raw_data[5] << 8) + raw_data[4];
}

int8_t L3GD20Gyroscope::read_temperature_8()
{
    return (int8_t)register_device.read_register(OUT_TEMP_ADDR);
}

float L3GD20Gyroscope::get_temperature_sensor_sensitivity()
{
    return -1.0f;
}

L3GD20Gyroscope::DataReadyInterruptMode L3GD20Gyroscope::_update_interrupt_register(int mode)
{
    DataReadyInterruptMode res;
    FIFOMode fifo_mode;

    switch (mode) {
    case 0:
        // disable interrupts
        register_device.update_register(CTRL_REG3_ADDR, 0x00, 0x0F);
        res = DRDY_DISABLE;
        break;
    case 1:
        // enable interrupt
        fifo_mode = get_fifo_mode();
        if (fifo_mode) {
            // watermark interrupt
            register_device.update_register(CTRL_REG3_ADDR, 0x04, 0x0F);
        } else {
            // DRDY interrupt
            register_device.update_register(CTRL_REG3_ADDR, 0x08, 0x0F);
        }
        res = DRDY_ENABLE;
        break;
    case 2:
        // update interrupt mode
        // note: it can be used if we switch off/of FIFO
        if (_update_interrupt_register(3) == DRDY_ENABLE) {
            res = _update_interrupt_register(1);
        } else {
            res = _update_interrupt_register(0);
        }
        break;
    case 3:
        // check if register enabled/disabled
        uint8_t val = register_device.read_register(CTRL_REG3_ADDR, 0x0F);
        res = val ? DRDY_ENABLE : DRDY_DISABLE;
        break;
    }
    return res;
}
