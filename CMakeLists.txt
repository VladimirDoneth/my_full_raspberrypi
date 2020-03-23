cmake_minimum_required(VERSION 3.13)
project(mqtt_ss1306_bme280_mpu6050)

set(CMAKE_CXX_STANDARD 20)

add_executable(super_app main.cpp
        lib_i2c/i2c_driver.h lib_i2c/i2c_driver.cpp
        lib_mpu6050/mpu6050_drive.h lib_mpu6050/mpu6050_drive.cpp
        lib_ssd1306/ssd1306_driver.cpp lib_ssd1306/ssd1306_driver.h lib_ssd1306/oled_fonts.h
        lib_bme280/BME280CalibrationData.h lib_bme280/BME280Data.h lib_bme280/BME280RawData.h lib_bme280/bme280.h lib_bme280/bme280.cpp)