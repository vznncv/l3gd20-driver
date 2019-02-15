# L3GD20 interface library

The library contains an gyroscope driver of the
[LSM303DLHC](https://www.st.com/en/mems-and-sensors/lsm303dlhc.html) chip for mbed-os.

The L3GD20 documentation can be found here:

- [datasheet](https://www.st.com/resource/en/datasheet/l3gd20.pdf)
- [application note](https://www.st.com/content/ccc/resource/technical/document/application_note/2c/d9/a7/f8/43/48/48/64/DM00119036.pdf/files/DM00119036.pdf/jcr:content/translations/en.DM00119036.pdf)

Library allows:

- set scale mode
- set output data rate
- enable and configure high pass filter
- configure low pass filter
- use FIFO to reduce communication between microcontroller and mems
- enable data ready interrupt line

## Driver usage

Typical interface usage contains the following steps:

1. create and configure `SPI` interface;
2. create `L3GD20Gyroscope` driver instances;
3. invoke `init` method. This method will perform basic device configuration, and set some default setting;
4. invoke driver method to configure L3GD20 for you purposes;
5. read data using `read_data`, `read_data_dps` or `read_data_16` methods.

The simple program that uses accelerometer with [STM32F3Discovery](https://www.st.com/en/evaluation-tools/stm32f3discovery.html)
board is shown bellow:

```
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
```

## Tests

The library contains some tests. To run them you should:

1. create a new project, with this library;
2. adjust `l3gd20dlhc-driver.test_*` pins in the `mbed_json.app` for SPI and interrupts if you don't use a STM32F3Discovery board; 
3. connect board;
4. run `mbed test --greentea --test-by-name "l3gd20-driver-tests-*"`.

Note: during tests the sensor shouldn't be moved.
