#include "bmi270_driver.h"
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(bmi270, LOG_LEVEL_INF);

/* Sensitivity: +/-8g range = 4096 LSB/g, +/-1000dps range = 32.768 LSB/dps */
#define ACCEL_SENSITIVITY  (9.80665f / 4096.0f)
#define GYRO_SENSITIVITY   (1.0f / 32.768f)

static int bmi270_reg_read(const struct device *dev, uint8_t reg, uint8_t *val, uint8_t len)
{
    return i2c_burst_read(dev, BMI270_I2C_ADDR, reg, val, len);
}

static int bmi270_reg_write(const struct device *dev, uint8_t reg, uint8_t val)
{
    return i2c_reg_write_byte(dev, BMI270_I2C_ADDR, reg, val);
}

bool bmi270_init(const struct device *i2c_dev)
{
    uint8_t chip_id = 0;

    if (bmi270_reg_read(i2c_dev, BMI270_CHIP_ID_REG, &chip_id, 1) != 0) {
        LOG_ERR("Failed to read BMI270 chip ID");
        return false;
    }

    if (chip_id != BMI270_CHIP_ID_VAL) {
        LOG_ERR("BMI270 chip ID mismatch: 0x%02x (expected 0x%02x)", chip_id, BMI270_CHIP_ID_VAL);
        return false;
    }

    LOG_INF("BMI270 detected (chip ID: 0x%02x)", chip_id);

    /* Soft reset */
    bmi270_reg_write(i2c_dev, BMI270_CMD_REG, 0xB6);
    k_msleep(10);

    /* Load config file (simplified - real implementation needs full init blob) */
    bmi270_reg_write(i2c_dev, BMI270_INIT_CTRL, 0x00);
    k_msleep(1);
    bmi270_reg_write(i2c_dev, BMI270_INIT_CTRL, 0x01);
    k_msleep(150);

    /* Accelerometer config: ODR=100Hz, OSR4, +/-8g */
    bmi270_reg_write(i2c_dev, BMI270_ACC_CONF, 0x28);
    bmi270_reg_write(i2c_dev, BMI270_ACC_RANGE, 0x02);

    /* Gyroscope config: ODR=100Hz, OSR4, +/-1000dps */
    bmi270_reg_write(i2c_dev, BMI270_GYR_CONF, 0x28);
    bmi270_reg_write(i2c_dev, BMI270_GYR_RANGE, 0x01);

    /* Power on accel + gyro */
    bmi270_reg_write(i2c_dev, BMI270_PWR_CTRL, 0x0E);
    k_msleep(50);

    LOG_INF("BMI270 initialized: 100Hz, +/-8g, +/-1000dps");
    return true;
}

bool bmi270_read(const struct device *i2c_dev, bmi270_data_t *data)
{
    uint8_t buf[12];

    if (bmi270_reg_read(i2c_dev, BMI270_ACC_X_LSB, buf, 6) != 0) {
        return false;
    }

    int16_t ax = (int16_t)(buf[1] << 8 | buf[0]);
    int16_t ay = (int16_t)(buf[3] << 8 | buf[2]);
    int16_t az = (int16_t)(buf[5] << 8 | buf[4]);

    if (bmi270_reg_read(i2c_dev, BMI270_GYR_X_LSB, buf, 6) != 0) {
        return false;
    }

    int16_t gx = (int16_t)(buf[1] << 8 | buf[0]);
    int16_t gy = (int16_t)(buf[3] << 8 | buf[2]);
    int16_t gz = (int16_t)(buf[5] << 8 | buf[4]);

    data->accel_x = ax * ACCEL_SENSITIVITY;
    data->accel_y = ay * ACCEL_SENSITIVITY;
    data->accel_z = az * ACCEL_SENSITIVITY;
    data->gyro_x = gx * GYRO_SENSITIVITY;
    data->gyro_y = gy * GYRO_SENSITIVITY;
    data->gyro_z = gz * GYRO_SENSITIVITY;

    return true;
}
