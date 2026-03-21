#include "motor_driver.h"
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <math.h>
#include <string.h>

LOG_MODULE_REGISTER(motor_driver, LOG_LEVEL_INF);

/* TB6600 stepper driver: PWM frequency = step frequency */
#define PWM_AZ_NODE  DT_NODELABEL(pwm_az)
#define PWM_EL_NODE  DT_NODELABEL(pwm_el)

static const struct pwm_dt_spec pwm_az = PWM_DT_SPEC_GET(DT_PATH(soc, timers3, pwm), 0);
static const struct pwm_dt_spec pwm_el = PWM_DT_SPEC_GET(DT_PATH(soc, timers4, pwm), 0);

static const struct gpio_dt_spec az_dir_gpio = GPIO_DT_SPEC_GET(DT_NODELABEL(az_dir), gpios);
static const struct gpio_dt_spec el_dir_gpio = GPIO_DT_SPEC_GET(DT_NODELABEL(el_dir), gpios);
static const struct gpio_dt_spec az_en_gpio  = GPIO_DT_SPEC_GET(DT_NODELABEL(az_en), gpios);
static const struct gpio_dt_spec el_en_gpio  = GPIO_DT_SPEC_GET(DT_NODELABEL(el_en), gpios);

static void set_axis_pwm(const struct pwm_dt_spec *pwm, float freq_hz)
{
    float abs_freq = fabsf(freq_hz);

    if (abs_freq < 1.0f) {
        pwm_set_pulse_dt(pwm, 0);
        return;
    }

    /* Period in nsec */
    uint32_t period_ns = (uint32_t)(1e9f / abs_freq);
    uint32_t pulse_ns = period_ns / 2; /* 50% duty cycle */

    pwm_set_dt(pwm, period_ns, pulse_ns);
}

bool motor_driver_init(motor_driver_t *drv)
{
    memset(drv, 0, sizeof(motor_driver_t));

    /* Configure direction GPIOs */
    if (gpio_is_ready_dt(&az_dir_gpio)) {
        gpio_pin_configure_dt(&az_dir_gpio, GPIO_OUTPUT_INACTIVE);
    }
    if (gpio_is_ready_dt(&el_dir_gpio)) {
        gpio_pin_configure_dt(&el_dir_gpio, GPIO_OUTPUT_INACTIVE);
    }
    if (gpio_is_ready_dt(&az_en_gpio)) {
        gpio_pin_configure_dt(&az_en_gpio, GPIO_OUTPUT_INACTIVE);
    }
    if (gpio_is_ready_dt(&el_en_gpio)) {
        gpio_pin_configure_dt(&el_en_gpio, GPIO_OUTPUT_INACTIVE);
    }

    LOG_INF("Motor driver initialized (TB6600 x2)");
    return true;
}

void motor_driver_set_command(motor_driver_t *drv, float az_freq_hz, float el_freq_hz,
                              bool az_dir, bool el_dir, bool estop)
{
    drv->emergency_stop = estop;

    if (estop) {
        motor_driver_stop(drv);
        return;
    }

    /* Enable motors */
    gpio_pin_set_dt(&az_en_gpio, 1);
    gpio_pin_set_dt(&el_en_gpio, 1);

    /* Set direction */
    gpio_pin_set_dt(&az_dir_gpio, az_dir ? 1 : 0);
    gpio_pin_set_dt(&el_dir_gpio, el_dir ? 1 : 0);

    /* Set PWM frequency */
    set_axis_pwm(&pwm_az, az_freq_hz);
    set_axis_pwm(&pwm_el, el_freq_hz);
}

void motor_driver_stop(motor_driver_t *drv)
{
    (void)drv;

    set_axis_pwm(&pwm_az, 0.0f);
    set_axis_pwm(&pwm_el, 0.0f);

    gpio_pin_set_dt(&az_en_gpio, 0);
    gpio_pin_set_dt(&el_en_gpio, 0);
}
