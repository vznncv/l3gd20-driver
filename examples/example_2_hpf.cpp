/**
 * Example of the L3GD20 usage with STM32F3Discovery board.
 *
 * Example of the high pass filter usage.
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
    // create separate spi instance
    SPI spi(PA_7, PA_6, PA_5);
    L3GD20Gyroscope gyroscope(&spi, PE_3);
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
    wait(2.5f);

    // gyroscope dps data
    float w_dps[3];
    int count = 0;

    while (true) {
        // read gyroscope data
        gyroscope.read_data_dps(w_dps);
        printf("%04d | wx: %+7.2f dps, wy: %+7.2f dps, xz: %+7.2f dps\n", count, w_dps[0], w_dps[1], w_dps[2]);

        led = !led;
        wait(0.05);
        count++;
    }
}
