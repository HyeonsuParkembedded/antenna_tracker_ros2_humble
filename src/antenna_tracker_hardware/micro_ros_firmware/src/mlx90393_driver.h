#ifndef MLX90393_DRIVER_H
#define MLX90393_DRIVER_H

#include <zephyr/device.h>
#include <stdbool.h>

#define MLX90393_I2C_ADDR 0x0C

/* Commands */
#define MLX90393_CMD_SB     0x10  /* Start burst mode */
#define MLX90393_CMD_SM     0x30  /* Start single measurement */
#define MLX90393_CMD_RM     0x40  /* Read measurement */
#define MLX90393_CMD_RT     0xF0  /* Reset */
#define MLX90393_CMD_EX     0x80  /* Exit mode */

/* Measurement flags */
#define MLX90393_ZYXT_ALL   0x0E  /* X + Y + Z */

typedef struct {
    float x;  /* uT */
    float y;
    float z;
} mlx90393_data_t;

bool mlx90393_init(const struct device *i2c_dev);
bool mlx90393_read(const struct device *i2c_dev, mlx90393_data_t *data);

#endif /* MLX90393_DRIVER_H */
