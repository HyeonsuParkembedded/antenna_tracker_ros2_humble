#include "mlx90393_driver.h"
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mlx90393, LOG_LEVEL_INF);

/* Sensitivity: default gain = 0.150 uT/LSB (HALLCONF=0x0C, gain_sel=7) */
#define MAG_SENSITIVITY 0.150f

static int mlx90393_cmd(const struct device *dev, uint8_t cmd, uint8_t *status)
{
    uint8_t tx = cmd;
    uint8_t rx = 0;

    int ret = i2c_write_read(dev, MLX90393_I2C_ADDR, &tx, 1, &rx, 1);
    if (status) {
        *status = rx;
    }
    return ret;
}

bool mlx90393_init(const struct device *i2c_dev)
{
    uint8_t status;

    /* Reset */
    if (mlx90393_cmd(i2c_dev, MLX90393_CMD_RT, &status) != 0) {
        LOG_ERR("MLX90393 reset failed");
        return false;
    }
    k_msleep(10);

    /* Exit any previous mode */
    mlx90393_cmd(i2c_dev, MLX90393_CMD_EX, &status);
    k_msleep(1);

    /* Start burst mode for X, Y, Z */
    uint8_t sb_cmd = MLX90393_CMD_SB | MLX90393_ZYXT_ALL;
    if (mlx90393_cmd(i2c_dev, sb_cmd, &status) != 0) {
        LOG_ERR("MLX90393 start burst failed");
        return false;
    }

    LOG_INF("MLX90393 initialized: burst mode XYZ");
    return true;
}

bool mlx90393_read(const struct device *i2c_dev, mlx90393_data_t *data)
{
    uint8_t tx = MLX90393_CMD_RM | MLX90393_ZYXT_ALL;
    uint8_t rx[7]; /* status + 3x int16 */

    if (i2c_write_read(i2c_dev, MLX90393_I2C_ADDR, &tx, 1, rx, 7) != 0) {
        return false;
    }

    int16_t raw_x = (int16_t)((rx[1] << 8) | rx[2]);
    int16_t raw_y = (int16_t)((rx[3] << 8) | rx[4]);
    int16_t raw_z = (int16_t)((rx[5] << 8) | rx[6]);

    data->x = raw_x * MAG_SENSITIVITY;
    data->y = raw_y * MAG_SENSITIVITY;
    data->z = raw_z * MAG_SENSITIVITY;

    return true;
}
