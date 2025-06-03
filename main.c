#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "bme280.h"
#include "esp_rom_sys.h"

// Add this if not defined:
#ifndef BME280_SLEEP_MODE
#define BME280_SLEEP_MODE 0x00
#endif

#ifndef BME280_FORCED_MODE
#define BME280_FORCED_MODE 0x01
#endif

// I2C configuration
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_SDA_IO          21
#define I2C_MASTER_SCL_IO          22
#define I2C_MASTER_FREQ_HZ         100000
#define BME280_ADDR                BME280_I2C_ADDR_PRIM // 0x76

// I2C read
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

// I2C write
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

// Delay (in microseconds)
void user_delay_us(uint32_t period, void *intf_ptr) {
    esp_rom_delay_us(period);
}

// Initialize I2C
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
    i2c_master_init();

    struct bme280_dev dev;
    uint8_t addr = BME280_ADDR;

    dev.intf = BME280_I2C_INTF;
    dev.intf_ptr = &addr;
    dev.read = user_i2c_read;
    dev.write = user_i2c_write;
    dev.delay_us = user_delay_us;

    if (bme280_init(&dev) != BME280_OK) {
        printf("BME280 init failed\n");
        return;
    }

    struct bme280_settings settings;
    settings.osr_h = BME280_OVERSAMPLING_1X;
    settings.osr_p = BME280_OVERSAMPLING_16X;
    settings.osr_t = BME280_OVERSAMPLING_2X;
    settings.filter = BME280_FILTER_COEFF_16;
    settings.standby_time = BME280_STANDBY_TIME_62_5_MS;

    if (bme280_set_sensor_settings(0xFF, &settings, &dev) != BME280_OK) {
        printf("Failed to set BME280 settings\n");
        return;
    }

    while (1) {
        bme280_set_sensor_mode(BME280_FORCED_MODE, &dev);
        dev.delay_us(40000, dev.intf_ptr);  // Wait for measurement

        struct bme280_data comp_data;
        if (bme280_get_sensor_data(BME280_ALL, &comp_data, &dev) == BME280_OK) {
            printf("Temp: %.2f °C, Pressure: %.2f hPa, Humidity: %.2f %%\n",
                   comp_data.temperature,
                   comp_data.pressure / 100.0,
                   comp_data.humidity);
        } else {
            printf("Failed to read sensor data\n");
        }

        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}
