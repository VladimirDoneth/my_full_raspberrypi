#include <iostream>
#include "lib_i2c/i2c_driver.h"
#include "lib_mpu6050/mpu6050_drive.h"
#include "lib_ssd1306/ssd1306_driver.h"
#include "lib_bme280/bme280.h"

int i2c_init();
int i2c_read_mpu(uint8_t reg, int read_len, uint8_t read_res[]);
int i2c_write_mpu(uint8_t reg, uint8_t data);

int i2c_read_ssd(uint8_t reg, int read_len, uint8_t read_res[]);
int i2c_write_ssd(uint8_t reg, uint8_t data);

int i2c_read_bme(uint8_t reg, int read_len, uint8_t read_res[]);
int i2c_write_bme(uint8_t reg, uint8_t data);

I2C_Driver *i2c_driver= new I2C_Driver(I2C_ADAPTER_1);


int main() {
    std::cout << "THIS IS MPU6050 activation" << std::endl;
    MPU6050_Drive *mpu6050Drive = new MPU6050_Drive(i2c_init, i2c_read_mpu, i2c_write_mpu);
    mpu6050Drive->init_mpu6050();

    std::cout << "THIS IS SSD1306 activation" << std::endl;
    SSD1306_Drive *ssd1306Drive = new SSD1306_Drive(i2c_init, i2c_read_ssd, i2c_write_ssd);
    ssd1306Drive->ssd1306_begin(SSD1306_SWITCHCAPVCC, SSD1306_I2C_ADDRESS);
    ssd1306Drive->ssd1306_display(); //Adafruit logo is visible
    ssd1306Drive->ssd1306_clearDisplay();
    sleep(2);

    std::cout << "THIS IS BME280 activation" << std::endl;
    BME280 *bme280_driver = new BME280("/dev/i2c-1", BME280_I2C_ADDRESS1, i2c_init, i2c_read_bme, i2c_write_bme);
    bme280_driver->init();
    bme280_driver->reset();
    printf("chip id  : 0x%02x\n", bme280_driver->getChipId());
    printf("chip ver : 0x%02x\n", bme280_driver->getChipVersion());

    bme280_driver->reset();
    bme280_driver->setPowerMode(BME280_NORMAL_MODE);
    bme280_driver->setTemperatureOversampling(BME280_ULTRAHIGHRESOLUTION_OVERSAMP_TEMPERATURE);
    bme280_driver->setPressureOversampling(BME280_ULTRAHIGHRESOLUTION_OVERSAMP_PRESSURE);
    bme280_driver->setHumidityOversampling(BME280_ULTRAHIGHRESOLUTION_OVERSAMP_HUMIDITY);
    bme280_driver->setIrrFilter(BME280_FILTER_COEFF_16);
    bme280_driver->setStandbyTime(BME280_STANDBY_TIME_250_MS);

    printf("---------------\n");
    printf("pw mode  : 0x%02x\n", bme280_driver->getPowerMode());
    printf("osrs_p   : 0x%02x\n", bme280_driver->getPressureOversampling());
    printf("osrs_t   : 0x%02x\n", bme280_driver->getTemperatureOversampling());
    printf("osrs_h   : 0x%02x\n", bme280_driver->getHumidityOversampling());
    printf("---------------\n");
    printf("filter   : 0x%02x\n", bme280_driver->getIrrFilter());
    printf("t_sb     : 0x%02x\n", bme280_driver->getStandbyTime());
    printf("---------------\n");
    printf("spi3w sts: 0x%02x\n", bme280_driver->getSpi3w());
    printf("measuring: 0x%02x\n", bme280_driver->getMeasuringStatus());
    printf("im_update: 0x%02x\n", bme280_driver->getImUpdateStatus());
    printf("---------------\n");

    while (1) {
        BMP280Data * bme280Data = bme280_driver->getBMP280Data();

        float accelX, accelY, accelZ, gyroX, gyroY, gyroZ;
        accelX = mpu6050Drive->get_accel_X();
        accelY = mpu6050Drive->get_accel_Y();
        accelZ = mpu6050Drive->get_accel_Z();

        gyroX = mpu6050Drive->get_gyro_X();
        gyroY = mpu6050Drive->get_gyro_Y();
        gyroZ = mpu6050Drive->get_gyro_Z();

        /*char json_str[80];
        sprintf(json_str, "{\"accelX\": %.3f, \"accelY\": %.3f, \"accelZ\": %.3f, \"gyroX\": %.3f, \"gyroY\": %.3f, \"gyroZ\": %.3f}",
                accelX, accelY, accelZ, gyroX, gyroY, gyroZ);
        */
        char disp_str[140];
        sprintf(disp_str, "aX:%.3f, aY:%.3f, aZ: %.3f\ngX%.3f, gY: %.3f\ngZ: %.3f\ntemperature: %.2f",
                accelX, accelY, accelZ, gyroX, gyroY, gyroZ, bme280Data->getTemperature());
        printf(disp_str);
        ssd1306Drive->ssd1306_drawString(disp_str);
        ssd1306Drive->ssd1306_display();
        ssd1306Drive->ssd1306_clearDisplay();
        sleep(1);
    }
    return 0;
}

int i2c_init() {
    return i2c_driver->open_i2c_file();
}

int i2c_read_mpu(uint8_t reg, int read_len, uint8_t read_res[]){
    return i2c_driver->read_buffer_i2c(Device_Address_mpu6050, reg, read_len, read_res);
}

int i2c_write_mpu(uint8_t reg, uint8_t data) {
    return i2c_driver->write_buffer_i2c(Device_Address_mpu6050, reg, data);
}



int i2c_read_ssd(uint8_t reg, int read_len, uint8_t read_res[]) {
    return i2c_driver->read_buffer_i2c(SSD1306_I2C_ADDRESS, reg, read_len, read_res);
}
int i2c_write_ssd(uint8_t reg, uint8_t data) {
    return i2c_driver->write_buffer_i2c(SSD1306_I2C_ADDRESS, reg, data);
}



int i2c_read_bme(uint8_t reg, int read_len, uint8_t read_res[]) {
    return  i2c_driver->read_buffer_i2c(BME280_ADDRESS, reg, read_len, read_res);
}

int i2c_write_bme(uint8_t reg, uint8_t data) {
    return i2c_driver->write_buffer_i2c(BME280_ADDRESS, reg, data);
}