#ifndef HALL_ENCODER_H
#define HALL_ENCODER_H

#include <zephyr/device.h>
#include <stdbool.h>
#include <stdint.h>

/* Encoder resolution: Hall sensor quadrature */
#define ENCODER_COUNTS_PER_REV  (200 * 4)  /* 200 poles * 4x quadrature */
#define GEAR_RATIO              5.0f
#define COUNTS_PER_DEG          ((ENCODER_COUNTS_PER_REV * GEAR_RATIO) / 360.0f)

typedef struct {
    int32_t count;
    int32_t prev_count;
    float angle_deg;
    float velocity_dps;
    bool valid;
    uint64_t last_read_ms;
} encoder_state_t;

typedef struct {
    encoder_state_t azimuth;
    encoder_state_t elevation;
} hall_encoder_t;

bool hall_encoder_init(hall_encoder_t *enc);
void hall_encoder_update(hall_encoder_t *enc, uint64_t now_ms);
void hall_encoder_reset(hall_encoder_t *enc);

#endif /* HALL_ENCODER_H */
