/**
 * Example of the L3GD20 usage with STM32F3Discovery board.
 *
 * Example of the high pass filter usage.
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
    // create separate spi instance
    SPI spi(L3GD20_SPI_MOSI_PIN, L3GD20_SPI_MISO_PIN, L3GD20_SPI_SCLK_PIN);
    L3GD20Gyroscope gyroscope(&spi, L3GD20_SPI_SSEL_PIN);
    // initialize device
    int err = gyroscope.init();
    if (err) {
        MBED_ERROR(MBED_ERROR_INITIALIZATION_FAILED, "Gyroscope initialization failed");
    }

    // configure filter
    gyroscope.set_high_pass_filter_cutoff_freq_mode(L3GD20Gyroscope::HPF_CF7);
    gyroscope.set_high_pass_filter_mode(L3GD20Gyroscope::HPF_ENABLE);

    // print gyroscope information
    printf("Output data rage: %.1f Hz\n", gyroscope.get_output_data_rate_hz());
    printf("Lower cutoff frequency: %6.3f Hz\n", gyroscope.get_high_pass_filter_cut_off_frequency());
    printf("Upper cutoff frequency: %6.3f Hz\n", gyroscope.get_low_pass_filter_cut_off_frequency());
    ThisThread::sleep_for(2500ms);

    // gyroscope dps data
    float w_dps[3];
    int count = 0;

    while (true) {
        // read gyroscope data
        gyroscope.read_data_dps(w_dps);
        printf("%04d | wx: %+7.2f dps, wy: %+7.2f dps, xz: %+7.2f dps\n", count, w_dps[0], w_dps[1], w_dps[2]);

        led = !led;
        ThisThread::sleep_for(50ms);
        count++;
    }
}
