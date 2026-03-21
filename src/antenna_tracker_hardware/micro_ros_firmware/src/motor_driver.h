#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <zephyr/device.h>
#include <stdbool.h>

typedef struct {
    const struct device *pwm_dev;
    const struct device *dir_gpio;
    uint32_t dir_pin;
    const struct device *en_gpio;
    uint32_t en_pin;
    uint32_t pwm_channel;
    bool enabled;
} motor_axis_t;

typedef struct {
    motor_axis_t azimuth;
    motor_axis_t elevation;
    bool emergency_stop;
} motor_driver_t;

bool motor_driver_init(motor_driver_t *drv);
void motor_driver_set_command(motor_driver_t *drv, float az_freq_hz, float el_freq_hz,
                              bool az_dir, bool el_dir, bool estop);
void motor_driver_stop(motor_driver_t *drv);

#endif /* MOTOR_DRIVER_H */
