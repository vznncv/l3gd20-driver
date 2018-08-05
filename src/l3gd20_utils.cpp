#include "l3gd20_utils.h"
using namespace l3gd20;

RegisterDevice::RegisterDevice(mbed::I2C* i2c_ptr)
{
    interface.i2c_ptr = i2c_ptr;
    state = I2C_DEVICE;
}

RegisterDevice::RegisterDevice(PinName sda, PinName scl)
{
    interface.i2c_ptr = new I2C(sda, scl);
    state = I2C_DEVICE | CLEANUP_I2C_PTR;
}

RegisterDevice::RegisterDevice(mbed::SPI* spi_ptr, PinName ssel)
{
    interface.spi_ptr = spi_ptr;
    spi_ptr->format(8, 3); // set data format
    state = SPI_DEVICE;
    if (ssel == NC) {
        spi_ssel_ptr = NULL;
    } else {
        spi_ssel_ptr = new DigitalOut(ssel, 1);
        state |= CLEANUP_SPI_SSEL;
    }
}

RegisterDevice::RegisterDevice(PinName mosi, PinName miso, PinName sclk, PinName ssel)
{
    interface.spi_ptr = new SPI(mosi, miso, sclk);
    interface.spi_ptr->frequency(10000000);
    interface.spi_ptr->format(8, 3);
    state = SPI_DEVICE | CLEANUP_SPI_PTR;
    if (ssel == NC) {
        spi_ssel_ptr = NULL;
    } else {
        spi_ssel_ptr = new DigitalOut(ssel, 1);
        state |= CLEANUP_SPI_SSEL;
    }
}

RegisterDevice::~RegisterDevice()
{
    if (state & CLEANUP_I2C_PTR) {
        delete interface.i2c_ptr;
    }
    if (state & CLEANUP_SPI_PTR) {
        delete interface.spi_ptr;
    }
    if (state & CLEANUP_SPI_SSEL) {
        delete spi_ssel_ptr;
    }
}

uint8_t RegisterDevice::read_register(uint8_t reg)
{
    uint8_t val;

    if (state & SPI_DEVICE) {
        // SPI is used
        reg = reg | 0x80; // read mode
        if (spi_ssel_ptr != NULL) {
            spi_ssel_ptr->write(0);
        }
        interface.spi_ptr->write(reg); // send register address
        val = interface.spi_ptr->write(0x00); // read register value
        if (spi_ssel_ptr != NULL) {
            spi_ssel_ptr->write(1);
        }
    } else {
        // the I2C is used
        int res;
        res = interface.i2c_ptr->write(I2C_ADDRESS, (char*)&reg, 1, true); // send register address
        if (res) {
            MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER_I2C, MBED_ERROR_CODE_WRITE_FAILED), "register reading failed");
        }
        res = interface.i2c_ptr->read(I2C_ADDRESS, (char*)&val, 1); // read register value
        if (res) {
            MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER_I2C, MBED_ERROR_CODE_READ_FAILED), "register reading failed");
        }
    }

    return val;
}

void RegisterDevice::write_register(uint8_t reg, uint8_t val)
{
    if (state & SPI_DEVICE) {
        // the SPI is used
        reg = reg & 0x7F; // write mode
        if (spi_ssel_ptr != NULL) {
            spi_ssel_ptr->write(0);
        }
        interface.spi_ptr->write(reg); // send register address
        interface.spi_ptr->write(val); // send value
        if (spi_ssel_ptr != NULL) {
            spi_ssel_ptr->write(1);
        }

    } else {
        // the I2C is used
        uint8_t data[2] = { reg, val };
        // write register address and value
        int res = interface.i2c_ptr->write(I2C_ADDRESS, (char*)data, 2);
        if (res) {
            MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER_I2C, MBED_ERROR_CODE_WRITE_FAILED), "register writing failed");
        }
    }
}

void RegisterDevice::update_register(uint8_t reg, uint8_t val, uint8_t mask)
{
    uint8_t reg_val = read_register(reg);
    reg_val &= ~mask;
    val = val & mask;
    reg_val |= val;
    write_register(reg, reg_val);
}

uint8_t RegisterDevice::read_register(uint8_t reg, uint8_t mask)
{
    return read_register(reg) & mask;
}

void RegisterDevice::read_registers(uint8_t reg, uint8_t* data, uint8_t length)
{

    if (state & SPI_DEVICE) {
        // the SPI is used
        reg |= 0x60; // read multiple bytes
        reg |= 0x80; // read mode
        if (spi_ssel_ptr != NULL) {
            spi_ssel_ptr->write(0);
        }
        interface.spi_ptr->write(reg); // send register address
        interface.spi_ptr->write(NULL, 0, (char*)data, length); // read data
        if (spi_ssel_ptr != NULL) {
            spi_ssel_ptr->write(1);
        }
    } else {
        // the I2C is used
        reg |= 0x80; // read multiple bytes
        int res;
        res = interface.i2c_ptr->write(I2C_ADDRESS, (char*)&reg, 1, true); // send register address
        if (res) {
            MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER_I2C, MBED_ERROR_CODE_WRITE_FAILED), "registers reading failed");
        }
        res = interface.i2c_ptr->read(I2C_ADDRESS, (char*)data, length); // read data
        if (res) {
            MBED_ERROR(MBED_MAKE_ERROR(MBED_MODULE_DRIVER_I2C, MBED_ERROR_CODE_READ_FAILED), "registers reading failed");
        }
    }
}
