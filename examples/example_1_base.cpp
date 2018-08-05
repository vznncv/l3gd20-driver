/**
 * Example of the L3GD20 usage with STM32F3Discovery board.
 *
 * Base example of the L3GD20 usage.
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

DigitalOut led(LED2);

int main()
{
    // create driver instance
    SPI spi(PA_7, PA_6, PA_5);
    L3GD20Gyroscope gyroscope(&spi, PE_3);
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
        wait(0.05);
        count++;
    }
}
