#ifndef BMI270_DRIVER_H
#define BMI270_DRIVER_H

#include <zephyr/device.h>
#include <stdbool.h>

#define BMI270_I2C_ADDR 0x68

#define BMI270_CHIP_ID_REG    0x00
#define BMI270_CHIP_ID_VAL    0x24
#define BMI270_ACC_X_LSB      0x0C
#define BMI270_GYR_X_LSB      0x12
#define BMI270_CMD_REG        0x7E
#define BMI270_PWR_CTRL       0x7D
#define BMI270_ACC_CONF       0x40
#define BMI270_GYR_CONF       0x42
#define BMI270_ACC_RANGE      0x41
#define BMI270_GYR_RANGE      0x43
#define BMI270_INIT_CTRL      0x59
#define BMI270_INIT_DATA      0x5E

typedef struct {
    float accel_x;  /* m/s^2 */
    float accel_y;
    float accel_z;
    float gyro_x;   /* deg/s */
    float gyro_y;
    float gyro_z;
} bmi270_data_t;

bool bmi270_init(const struct device *i2c_dev);
bool bmi270_read(const struct device *i2c_dev, bmi270_data_t *data);

#endif /* BMI270_DRIVER_H */
