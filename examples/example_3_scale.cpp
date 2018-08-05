/**
 * Example of the L3GD20 usage with STM32F3Discovery board.
 *
 * Scale demonstration.
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

void print_axis_val(const char* axis_name, int16_t value)
{
    // convert value binary representation to show precision/resolution settings
    int sign = 1;
    if (value < 0) {
        sign = -1;
        value = -value;
    }

    char buff[32];
    int pos = 0;
    for (int i = 0; i < 16; i++) {
        buff[pos] = value & 0x8000 ? '1' : '0';
        value <<= 1;
        pos++;
        if (i % 4 == 3) {
            buff[pos] = '_';
            pos++;
        }
    }
    buff[pos - 1] = '\0';

    printf("%s: %c0b%s (raw)\n", axis_name, sign >= 0 ? '+' : '-', buff);
}

void read_and_print_data(L3GD20Gyroscope* gyro)
{
    int16_t data_16[3];
    // read data
    gyro->read_data_16(data_16);
    printf("--------------------------------\n");
    print_axis_val("wx", data_16[0]);
    print_axis_val("wy", data_16[1]);
    print_axis_val("wz", data_16[2]);
}

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

    L3GD20Gyroscope::FullScale full_scale_modes[4] = {
        L3GD20Gyroscope::FULL_SCALE_250,
        L3GD20Gyroscope::FULL_SCALE_500,
        L3GD20Gyroscope::FULL_SCALE_1000,
        L3GD20Gyroscope::FULL_SCALE_2000
    };
    const char* scale_mode_names[4] = {
        "Full scale:  250 dps",
        "Full scale:  500 dps",
        "Full scale: 1000 dps",
        "Full scale: 2000 dps"
    };

    while (true) {
        for (int i = 0; i < 4; i++) {
            gyroscope.set_full_scale(full_scale_modes[i]);
            printf("\n%s\n", scale_mode_names[i]);
            wait(0.5f);
            for (int j = 0; j < 3; j++) {
                read_and_print_data(&gyroscope);
                wait(0.2f);
                led = !led;
            }
            wait(1.5f);
        }
    }
}
