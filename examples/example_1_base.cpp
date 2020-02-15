/**
 * Example of the L3GD20 usage with STM32F3Discovery board.
 *
 * Base example of the L3GD20 usage.
 */
#include "l3gd20_driver.h"
#include "math.h"
#include "mbed.h"

/**
 * Pin map:
 *
 * - L3GD20_SPI_MOSI_PIN - SPI MOSI of the L3GD20
 * - L3GD20_SPI_MISO_PIN - SPI MISO of the L3GD20
 * - L3GD20_SPI_SCLK_PIN - SPI SCLK of the L3GD20
 * - L3GD20_SPI_SSEL_PIN - SPI SSEL of the L3GD20
 */
#define L3GD20_SPI_MOSI_PIN PA_7
#define L3GD20_SPI_MISO_PIN PA_6
#define L3GD20_SPI_SCLK_PIN PA_5
#define L3GD20_SPI_SSEL_PIN PE_3

DigitalOut led(LED2);

int main()
{
    // create driver instance
    SPI spi(L3GD20_SPI_MOSI_PIN, L3GD20_SPI_MISO_PIN, L3GD20_SPI_SCLK_PIN);
    L3GD20Gyroscope gyroscope(&spi, L3GD20_SPI_SSEL_PIN);
    // initialize device
    int err = gyroscope.init();
    if (err) {
        MBED_ERROR(MBED_ERROR_INITIALIZATION_FAILED, "Gyroscope initialization failed");
    }

    // gyroscope dps data
    float w_dps[3];
    int count = 0;

    while (true) {
        // read gyroscope data
        gyroscope.read_data_dps(w_dps);
        float x = w_dps[0];
        float y = w_dps[1];
        float z = w_dps[2];
        printf("%04d | wx: %+7.2f dps, wy: %+7.2f dps, wz: %+7.2f dps\n", count, x, y, z);

        led = !led;
        ThisThread::sleep_for(50);
        count++;
    }
}
