#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/magnetic_field.h>
#include <sensor_msgs/msg/nav_sat_fix.h>
#include <antenna_tracker_msgs/msg/encoder_feedback.h>
#include <antenna_tracker_msgs/msg/motor_command.h>

#include <rmw_microros/rmw_microros.h>

#include "bmi270_driver.h"
#include "mlx90393_driver.h"
#include "gps_parser.h"
#include "hall_encoder.h"
#include "motor_driver.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

#define LOOP_PERIOD_MS  10   /* 100 Hz */
#define GPS_PERIOD_MS   1000 /* 1 Hz */

/* I2C device */
#define I2C_NODE DT_NODELABEL(i2c1)
static const struct device *i2c_dev;

/* GPS UART */
#define GPS_UART_NODE DT_NODELABEL(usart3)
static const struct device *gps_uart_dev;

/* Subsystem state */
static hall_encoder_t encoder;
static motor_driver_t motor;
static gps_parser_t gps;

/* micro-ROS entities */
static rcl_publisher_t pub_imu;
static rcl_publisher_t pub_mag;
static rcl_publisher_t pub_gps;
static rcl_publisher_t pub_encoder;
static rcl_subscription_t sub_motor_cmd;

static sensor_msgs__msg__Imu msg_imu;
static sensor_msgs__msg__MagneticField msg_mag;
static sensor_msgs__msg__NavSatFix msg_gps;
static antenna_tracker_msgs__msg__EncoderFeedback msg_encoder;
static antenna_tracker_msgs__msg__MotorCommand msg_motor_cmd;

static rclc_executor_t executor;
static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rcl_timer_t timer_100hz;
static rcl_timer_t timer_1hz;

/* Motor command callback */
static void motor_cmd_callback(const void *msgin)
{
    const antenna_tracker_msgs__msg__MotorCommand *cmd =
        (const antenna_tracker_msgs__msg__MotorCommand *)msgin;

    motor_driver_set_command(&motor,
                             (float)cmd->az_frequency_hz,
                             (float)cmd->el_frequency_hz,
                             cmd->az_direction,
                             cmd->el_direction,
                             cmd->emergency_stop);
}

/* 100Hz timer: IMU + Mag + Encoder */
static void timer_100hz_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)timer;
    (void)last_call_time;

    uint64_t now_ms = k_uptime_get();

    /* Read BMI270 */
    bmi270_data_t imu_data;
    if (bmi270_read(i2c_dev, &imu_data)) {
        msg_imu.header.stamp.sec = (int32_t)(now_ms / 1000);
        msg_imu.header.stamp.nanosec = (uint32_t)((now_ms % 1000) * 1000000);

        msg_imu.linear_acceleration.x = imu_data.accel_x;
        msg_imu.linear_acceleration.y = imu_data.accel_y;
        msg_imu.linear_acceleration.z = imu_data.accel_z;

        msg_imu.angular_velocity.x = imu_data.gyro_x * 0.017453292f; /* deg/s -> rad/s */
        msg_imu.angular_velocity.y = imu_data.gyro_y * 0.017453292f;
        msg_imu.angular_velocity.z = imu_data.gyro_z * 0.017453292f;

        rcl_publish(&pub_imu, &msg_imu, NULL);
    }

    /* Read MLX90393 */
    mlx90393_data_t mag_data;
    if (mlx90393_read(i2c_dev, &mag_data)) {
        msg_mag.header.stamp = msg_imu.header.stamp;

        msg_mag.magnetic_field.x = mag_data.x * 1e-6; /* uT -> T */
        msg_mag.magnetic_field.y = mag_data.y * 1e-6;
        msg_mag.magnetic_field.z = mag_data.z * 1e-6;

        rcl_publish(&pub_mag, &msg_mag, NULL);
    }

    /* Read Hall Encoders */
    hall_encoder_update(&encoder, now_ms);
    msg_encoder.header.stamp = msg_imu.header.stamp;
    msg_encoder.az_encoder_count = encoder.azimuth.count;
    msg_encoder.el_encoder_count = encoder.elevation.count;
    msg_encoder.az_angle_deg = encoder.azimuth.angle_deg;
    msg_encoder.el_angle_deg = encoder.elevation.angle_deg;
    msg_encoder.az_velocity_dps = encoder.azimuth.velocity_dps;
    msg_encoder.el_velocity_dps = encoder.elevation.velocity_dps;
    msg_encoder.az_valid = encoder.azimuth.valid;
    msg_encoder.el_valid = encoder.elevation.valid;

    rcl_publish(&pub_encoder, &msg_encoder, NULL);
}

/* GPS UART ISR callback */
static void gps_uart_cb(const struct device *dev, void *user_data)
{
    (void)user_data;
    uint8_t c;

    while (uart_fifo_read(dev, &c, 1) > 0) {
        gps_parser_feed_char(&gps, (char)c, k_uptime_get());
    }
}

/* 1Hz timer: GPS publish */
static void timer_1hz_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    (void)timer;
    (void)last_call_time;

    if (gps_parser_has_fix(&gps)) {
        const gps_fix_t *fix = gps_parser_get_fix(&gps);
        uint64_t now_ms = k_uptime_get();

        msg_gps.header.stamp.sec = (int32_t)(now_ms / 1000);
        msg_gps.header.stamp.nanosec = (uint32_t)((now_ms % 1000) * 1000000);

        msg_gps.latitude = fix->latitude;
        msg_gps.longitude = fix->longitude;
        msg_gps.altitude = fix->altitude;
        msg_gps.status.status = (fix->fix_quality > 0) ? 0 : -1;
        msg_gps.status.service = 1; /* GPS */

        rcl_publish(&pub_gps, &msg_gps, NULL);
    }
}

void main(void)
{
    LOG_INF("Antenna Tracker micro-ROS Firmware starting...");

    /* Initialize I2C */
    i2c_dev = DEVICE_DT_GET(I2C_NODE);
    if (!device_is_ready(i2c_dev)) {
        LOG_ERR("I2C device not ready");
        return;
    }

    /* Initialize sensors */
    if (!bmi270_init(i2c_dev)) {
        LOG_ERR("BMI270 init failed");
    }
    if (!mlx90393_init(i2c_dev)) {
        LOG_ERR("MLX90393 init failed");
    }

    /* Initialize GPS UART */
    gps_parser_init(&gps);
    gps_uart_dev = DEVICE_DT_GET(GPS_UART_NODE);
    if (device_is_ready(gps_uart_dev)) {
        uart_irq_callback_set(gps_uart_dev, gps_uart_cb);
        uart_irq_rx_enable(gps_uart_dev);
        LOG_INF("GPS UART ready");
    } else {
        LOG_WRN("GPS UART not available");
    }

    /* Initialize encoders */
    hall_encoder_init(&encoder);

    /* Initialize motor driver */
    motor_driver_init(&motor);

    /* micro-ROS transport (USB CDC) */
    rmw_uros_set_custom_transport(
        true, NULL,
        NULL, NULL, NULL, NULL);

    allocator = rcl_get_default_allocator();

    /* Init support */
    rclc_support_init(&support, 0, NULL, &allocator);

    /* Create node */
    rclc_node_init_default(&node, "antenna_tracker_mcu", "", &support);

    /* Publishers */
    rclc_publisher_init_best_effort(
        &pub_imu, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "/imu/raw");

    rclc_publisher_init_best_effort(
        &pub_mag, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, MagneticField),
        "/magnetic_field");

    rclc_publisher_init_best_effort(
        &pub_gps, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, NavSatFix),
        "/gps/fix");

    rclc_publisher_init_best_effort(
        &pub_encoder, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(antenna_tracker_msgs, msg, EncoderFeedback),
        "/antenna/encoder_feedback");

    /* Subscriber */
    rclc_subscription_init_best_effort(
        &sub_motor_cmd, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(antenna_tracker_msgs, msg, MotorCommand),
        "/antenna/motor_cmd");

    /* Timers */
    rclc_timer_init_default(&timer_100hz, &support, RCL_MS_TO_NS(LOOP_PERIOD_MS),
                            timer_100hz_callback);
    rclc_timer_init_default(&timer_1hz, &support, RCL_MS_TO_NS(GPS_PERIOD_MS),
                            timer_1hz_callback);

    /* Executor: 1 subscription + 2 timers */
    rclc_executor_init(&executor, &support.context, 3, &allocator);
    rclc_executor_add_subscription(&executor, &sub_motor_cmd, &msg_motor_cmd,
                                   &motor_cmd_callback, ON_NEW_DATA);
    rclc_executor_add_timer(&executor, &timer_100hz);
    rclc_executor_add_timer(&executor, &timer_1hz);

    LOG_INF("micro-ROS executor running");

    /* Spin forever */
    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(1));
        k_msleep(1);
    }
}
