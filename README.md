# ESP32-BME280-Test
ESP32 + BME280 sensor example using ESP-IDF and Bosch's official bme280_driver (I2C mode)

This project demonstrates how to interface a BME280 environmental sensor with an ESP32 using the ESP-IDF framework (v5.4.1) and Bosch's official bme280_driver. It reads and prints temperature, humidity, and pressure data over the I2C interface at regular intervals.  
Unlike the older bme280 component, this project uses the lightweight, low-level bme280_driver directly, offering better control and flexibility. It’s ideal for learning how to integrate third-party drivers with ESP-IDF or for use in custom sensor-based applications.

Output:  
![image](https://github.com/user-attachments/assets/96768579-8380-4630-ad19-f0eb38195d2b)

Challenges:  
Missing macros and settings: Constants like BME280_NORMAL_MODE, BME280_FORCED_MODE, and settings bitmasks (BME280_OSR_*, BME280_FILTER_*) are not enabled by default. They need to be defined or manually implemented using appropriate values based on the datasheet and examples.  
Custom delay function: The driver expects a delay_us() function. ESP-IDF does not provide ets_delay_us() in user apps, so it must be replaced with esp_rom_delay_us() and requires including esp_rom_sys.h.  
Data not updating in NORMAL mode: Using BME280_NORMAL_MODE requires proper timing management. For simplicity, this example uses BME280_FORCED_MODE, which manually triggers a measurement each loop.
