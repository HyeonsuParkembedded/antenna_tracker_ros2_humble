#include "hall_encoder.h"
#include <zephyr/drivers/counter.h>
#include <zephyr/logging/log.h>
#include <string.h>

LOG_MODULE_REGISTER(hall_encoder, LOG_LEVEL_INF);

/* Timer counter devices (TIM1 for azimuth, TIM8 for elevation) */
static const struct device *tim1_dev;
static const struct device *tim8_dev;

static int32_t read_timer_count(const struct device *dev)
{
    uint32_t ticks = 0;
    if (dev) {
        counter_get_value(dev, &ticks);
    }
    return (int32_t)ticks;
}

static void update_axis(encoder_state_t *axis, int32_t raw_count, uint64_t now_ms)
{
    axis->prev_count = axis->count;
    axis->count = raw_count;
    axis->angle_deg = (float)raw_count / COUNTS_PER_DEG;

    float dt = 0.0f;
    if (axis->last_read_ms > 0) {
        dt = (now_ms - axis->last_read_ms) / 1000.0f;
    }

    if (dt > 0.0f) {
        float delta_deg = (float)(axis->count - axis->prev_count) / COUNTS_PER_DEG;
        axis->velocity_dps = delta_deg / dt;
    }

    axis->last_read_ms = now_ms;
    axis->valid = true;
}

bool hall_encoder_init(hall_encoder_t *enc)
{
    memset(enc, 0, sizeof(hall_encoder_t));

    tim1_dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(encoder_az));
    tim8_dev = DEVICE_DT_GET_OR_NULL(DT_NODELABEL(encoder_el));

    if (tim1_dev && device_is_ready(tim1_dev)) {
        counter_start(tim1_dev);
        LOG_INF("Azimuth encoder (TIM1) ready");
    } else {
        LOG_WRN("Azimuth encoder not available");
        tim1_dev = NULL;
    }

    if (tim8_dev && device_is_ready(tim8_dev)) {
        counter_start(tim8_dev);
        LOG_INF("Elevation encoder (TIM8) ready");
    } else {
        LOG_WRN("Elevation encoder not available");
        tim8_dev = NULL;
    }

    return (tim1_dev != NULL) || (tim8_dev != NULL);
}

void hall_encoder_update(hall_encoder_t *enc, uint64_t now_ms)
{
    if (tim1_dev) {
        int32_t az_count = read_timer_count(tim1_dev);
        update_axis(&enc->azimuth, az_count, now_ms);
    }

    if (tim8_dev) {
        int32_t el_count = read_timer_count(tim8_dev);
        update_axis(&enc->elevation, el_count, now_ms);
    }
}

void hall_encoder_reset(hall_encoder_t *enc)
{
    enc->azimuth.count = 0;
    enc->azimuth.prev_count = 0;
    enc->azimuth.angle_deg = 0.0f;
    enc->azimuth.velocity_dps = 0.0f;

    enc->elevation.count = 0;
    enc->elevation.prev_count = 0;
    enc->elevation.angle_deg = 0.0f;
    enc->elevation.velocity_dps = 0.0f;
}
