#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "bme280.h"
#include "esp_rom_sys.h"   // For esp_rom_delay_us()

#ifndef BME280_SLEEP_MODE
#define BME280_SLEEP_MODE 0x00
#endif

#ifndef BME280_FORCED_MODE
#define BME280_FORCED_MODE 0x01
#endif

// --- I2C configuration for ESP32 ---
#define I2C_MASTER_NUM             I2C_NUM_0            // Use I2C port 0
#define I2C_MASTER_SDA_IO          21                   // GPIO number for SDA line
#define I2C_MASTER_SCL_IO          22                   // GPIO number for SCL line
#define I2C_MASTER_FREQ_HZ         100000               // 100kHz I2C clock speed
#define BME280_ADDR                BME280_I2C_ADDR_PRIM // 0x76

// --- I2C read function ---
// This is called by the BME280 driver to read sensor registers via I2C
BME280_INTF_RET_TYPE user_i2c_read(uint8_t reg_addr, uint8_t *data, uint32_t len, void *intf_ptr) {
    uint8_t addr = *(uint8_t *)intf_ptr;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret == ESP_OK ? BME280_OK : BME280_E_COMM_FAIL;
}

// --- I2C write function ---
// Called by BME280 driver to write sensor registers
BME280_INTF_RET_TYPE user_i2c_write(uint8_t reg_addr, const uint8_t *data, uint32_t len, void *intf_ptr) {
    uint8_t addr = *(uint8_t *)intf_ptr;

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write(cmd, (uint8_t *)data, len, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    return ret == ESP_OK ? BME280_OK : BME280_E_COMM_FAIL;
}

// --- Delay function ---
// Used by BME280 driver to wait for sensor timing
void user_delay_us(uint32_t period, void *intf_ptr) {
    esp_rom_delay_us(period);   // Delay in microseconds
}

// --- Initialize I2C hardware ---
// Sets up I2C peripheral for master mode with specified pins and clock
void i2c_master_init(void) {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    i2c_param_config(I2C_MASTER_NUM, &conf);
    i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0);
}

void app_main(void) {
    // 1. Initialize I2C interface for sensor communication
    i2c_master_init();

    // 2. Prepare BME280 device struct and interface settings
    struct bme280_dev dev;
    uint8_t addr = BME280_ADDR;

    dev.intf = BME280_I2C_INTF;     // Interface type is I2C
    dev.intf_ptr = &addr;           // Pointer to device address
    dev.read = user_i2c_read;       // Assign I2C read function
    dev.write = user_i2c_write;     // Assign I2C write function
    dev.delay_us = user_delay_us;   // Assign delay function

    // 3. Initialize the sensor and check for errors
    if (bme280_init(&dev) != BME280_OK) {
        printf("BME280 init failed\n");
        return;
    }

    // 4. Configure sensor settings (oversampling, filter, standby time)
    struct bme280_settings settings;
    settings.osr_h = BME280_OVERSAMPLING_1X;          // Humidity oversampling x1
    settings.osr_p = BME280_OVERSAMPLING_16X;         // Pressure oversampling x16 for better resolution
    settings.osr_t = BME280_OVERSAMPLING_2X;          // Temperature oversampling x2
    settings.filter = BME280_FILTER_COEFF_16;         // IIR filter coefficient 16 for noise reduction
    settings.standby_time = BME280_STANDBY_TIME_62_5_MS; // Standby time between measurements

    // Apply all settings (bitmask 0xFF applies everything)
    if (bme280_set_sensor_settings(0xFF, &settings, &dev) != BME280_OK) {
        printf("Failed to set BME280 settings\n");
        return;
    }

    // 5. Main loop: read sensor data every 2 seconds using FORCED mode
    while (1) {
        // Trigger a measurement
        bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);

        // Wait for measurement to complete (~40ms typical)
        dev.delay_us(40000, dev.intf_ptr);

        // Structure to hold sensor data
        struct bme280_data comp_data;

        // Read sensor data
        if (bme280_get_sensor_data(BME280_ALL, &comp_data, &dev) == BME280_OK) {
            printf("Temp: %.2f °C, Pressure: %.2f hPa, Humidity: %.2f %%\n",
                   comp_data.temperature,
                   comp_data.pressure / 100.0,
                   comp_data.humidity);
        } else {
            printf("Failed to read sensor data\n");
        }

        // Wait 2 seconds before next reading
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
