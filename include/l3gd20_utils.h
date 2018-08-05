#ifndef L3GD20_UTILS_H
#define L3GD20_UTILS_H
#include "mbed.h"

namespace l3gd20 {

/**
 * Inner helper class for L3GD20 driver interface.
 *
 * It shouldn't be used directly.
 */
class RegisterDevice : public NonCopyable<RegisterDevice> {
public:
    /**
     * Constructor
     *
     * @param i2c_ptr I2C interface
     */
    RegisterDevice(I2C* i2c_ptr);

    /**
     * Constructor.
     *
     * @param sda I2C data line pin
     * @param scl I2C clock line pin
     */
    RegisterDevice(PinName sda, PinName scl);

    /**
     * Constructor.
     *
     * @param spi_ptr SPI interface
     */
    RegisterDevice(SPI* spi_ptr, PinName ssel);

    /**
     * Constructor.
     *
     * @param mosi SPI mosi pin
     * @param miso SPI miso pin
     * @param sclk SPI sclk pin
     * @param ssel SPI ssel pin
     */
    RegisterDevice(PinName mosi, PinName miso, PinName sclk, PinName ssel);

    virtual ~RegisterDevice();

    /**
     * Read device register.
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

    /**
     * Update specified register.
     *
     * Only bits that are selected by mask will be updated.
     *
     * @param reg register address
     * @param val value to set
     * @param mask value mask
     */
    void update_register(uint8_t reg, uint8_t val, uint8_t mask);

    /**
     * Version of the read_register method with mask.
     *
     * Any bit in the result that corresponds zero bits in the mask will be set to 0.
     *
     * @param reg register address
     * @param mask mask
     * @return masked register value
     */
    uint8_t read_register(uint8_t reg, uint8_t mask);

    /**
     * Read several registers, starting with address \p reg.
     *
     * @note
     * This method cannot be invoked from an ISR context.
     *
     * @param reg
     * @param data
     * @param length
     */
    void read_registers(uint8_t reg, uint8_t* data, uint8_t length);

private:
    // helper variable with state flags
    uint8_t state;
    enum StateFlags {
        SPI_DEVICE = 0x01,
        I2C_DEVICE = 0x02,
        CLEANUP_I2C_PTR = 0x04,
        CLEANUP_SPI_PTR = 0x08,
        CLEANUP_SPI_SSEL = 0x10
    };

    // spi/i2c data
    union Interface {
        SPI* spi_ptr;
        I2C* i2c_ptr;
    };
    Interface interface;

    // I2C address (assume SDO pin is set to 0)
    static const uint8_t I2C_ADDRESS = 0xDA;

    DigitalOut* spi_ssel_ptr;
};
}
#endif // L3GD20_UTILS_H
