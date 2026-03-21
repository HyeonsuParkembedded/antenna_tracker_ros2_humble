#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microxrcedds_c/config.h>
#include <rmw_microros/rmw_microros.h>

#include <antenna_tracker_msgs/msg/motor_command.h>
#include <antenna_tracker_msgs/msg/encoder_feedback.h>
#include <sensor_msgs/msg/imu.h>

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printk("Failed status on line %d: %d. Aborting.\n",__LINE__,(int)temp_rc); return 1;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){printk("Failed status on line %d: %d. Continuing.\n",__LINE__,(int)temp_rc);}}

extern bool zephyr_transport_open(struct uxrCustomTransport * transport);
extern bool zephyr_transport_close(struct uxrCustomTransport * transport);
extern size_t zephyr_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
extern size_t zephyr_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

rcl_publisher_t encoder_pub;
rcl_publisher_t imu_pub;
rcl_subscription_t motor_sub;

antenna_tracker_msgs__msg__EncoderFeedback encoder_msg;
sensor_msgs__msg__Imu imu_msg;
antenna_tracker_msgs__msg__MotorCommand motor_msg;

/* Hardware Peripherals */
static const struct gpio_dt_spec az_step = GPIO_DT_SPEC_GET(DT_NODELABEL(az_step), gpios);
static const struct gpio_dt_spec az_dir  = GPIO_DT_SPEC_GET(DT_NODELABEL(az_dir), gpios);
static const struct gpio_dt_spec az_en   = GPIO_DT_SPEC_GET(DT_NODELABEL(az_en), gpios);

static const struct gpio_dt_spec el_step = GPIO_DT_SPEC_GET(DT_NODELABEL(el_step), gpios);
static const struct gpio_dt_spec el_dir  = GPIO_DT_SPEC_GET(DT_NODELABEL(el_dir), gpios);
static const struct gpio_dt_spec el_en   = GPIO_DT_SPEC_GET(DT_NODELABEL(el_en), gpios);

const struct device *i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c1));

#define AS5600_AZ_ADDR 0x36
#define BNO055_ADDR 0x28

/* Global Motor Tracking Variables */
float current_az_freq = 0.0;
float current_el_freq = 0.0;

void motor_cmd_callback(const void * msgin)
{
	const antenna_tracker_msgs__msg__MotorCommand * msg = (const antenna_tracker_msgs__msg__MotorCommand *)msgin;
    
    if (msg->emergency_stop) {
        gpio_pin_set_dt(&az_en, 0); // Disable motor (Assuming Active Low)
        gpio_pin_set_dt(&el_en, 0);
        current_az_freq = 0.0;
        current_el_freq = 0.0;
        return;
    }

    gpio_pin_set_dt(&az_en, 1);
    gpio_pin_set_dt(&el_en, 1);

    gpio_pin_set_dt(&az_dir, msg->az_direction ? 1 : 0);
    gpio_pin_set_dt(&el_dir, msg->el_direction ? 1 : 0);

    current_az_freq = msg->az_frequency_hz;
    current_el_freq = msg->el_frequency_hz;
}

/* K_Timer for Stepper motor stepping (~10kHz logic) */
void stepper_timer_isr(struct k_timer *dummy) {
    static float az_accum = 0.0;
    static float el_accum = 0.0;
    float dt = 0.0001; // 100 us
    
    az_accum += current_az_freq * dt;
    if (az_accum >= 1.0) {
        gpio_pin_toggle_dt(&az_step);
        az_accum -= 1.0;
    }

    el_accum += current_el_freq * dt;
    if (el_accum >= 1.0) {
        gpio_pin_toggle_dt(&el_step);
        el_accum -= 1.0;
    }
}
K_TIMER_DEFINE(stepper_timer, stepper_timer_isr, NULL);

/* Sensor read functions */
float read_as5600_angle(uint8_t addr) {
    if (!device_is_ready(i2c_dev)) return 0.0;
    uint8_t reg = 0x0E; // RAW ANGLE register in AS5600
    uint8_t buf[2] = {0};
    if (i2c_write_read(i2c_dev, addr, &reg, 1, buf, 2) == 0) {
        uint16_t raw_angle = (buf[0] << 8) | buf[1];
        return ((float)raw_angle / 4096.0f) * 360.0f;
    }
    return 0.0;
}

void read_bno055_imu(sensor_msgs__msg__Imu *msg) {
    if (!device_is_ready(i2c_dev)) return;
    // Real implementation requires BNO055 standard registers implementation
    msg->linear_acceleration.x = 0.0;
    msg->angular_velocity.x = 0.0;
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
	(void) last_call_time;
	if (timer != NULL) {
        // Read AS5600 Encoders
        encoder_msg.az_angle_deg = read_as5600_angle(AS5600_AZ_ADDR); 
        encoder_msg.el_angle_deg = 0.0; 
        encoder_msg.az_valid = true;
        encoder_msg.el_valid = false;
		RCSOFTCHECK(rcl_publish(&encoder_pub, &encoder_msg, NULL));

        // Read IMU
        imu_msg.header.stamp.sec = k_uptime_get() / 1000;
        read_bno055_imu(&imu_msg);
		RCSOFTCHECK(rcl_publish(&imu_pub, &imu_msg, NULL));
	}
}

static void init_hardware() {
    gpio_pin_configure_dt(&az_step, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&az_dir, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&az_en, GPIO_OUTPUT_INACTIVE);

    gpio_pin_configure_dt(&el_step, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&el_dir, GPIO_OUTPUT_INACTIVE);
    gpio_pin_configure_dt(&el_en, GPIO_OUTPUT_INACTIVE);

    k_timer_start(&stepper_timer, K_USEC(100), K_USEC(100)); // 10kHz timer
}

int main(void)
{
	printk("Starting Micro-ROS Antenna Tracker Firmware with Hardware Interface...\n");
    init_hardware();

	rmw_uros_set_custom_transport(
		true,
		NULL,
		zephyr_transport_open,
		zephyr_transport_close,
		zephyr_transport_write,
		zephyr_transport_read
	);

	rcl_allocator_t allocator = rcl_get_default_allocator();
	rclc_support_t support;

	while(rmw_uros_ping_agent(1000, 1) != RMW_RET_OK) {
		k_msleep(1000);
	}

	RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

	rcl_node_t node;
	RCCHECK(rclc_node_init_default(&node, "tracker_firmware", "", &support));

	RCCHECK(rclc_publisher_init_default(
		&encoder_pub,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(antenna_tracker_msgs, msg, EncoderFeedback),
		"/antenna/encoder_feedback"));

	RCCHECK(rclc_publisher_init_default(
		&imu_pub,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
		"/imu/raw"));

	RCCHECK(rclc_subscription_init_default(
		&motor_sub,
		&node,
		ROSIDL_GET_MSG_TYPE_SUPPORT(antenna_tracker_msgs, msg, MotorCommand),
		"/antenna/motor_cmd"));

	rcl_timer_t timer;
	RCCHECK(rclc_timer_init_default2(
		&timer,
		&support,
		RCL_MS_TO_NS(10), // 100Hz
		timer_callback,
        true));

	rclc_executor_t executor;
	RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
	RCCHECK(rclc_executor_add_subscription(&executor, &motor_sub, &motor_msg, &motor_cmd_callback, ON_NEW_DATA));
	RCCHECK(rclc_executor_add_timer(&executor, &timer));

	while (1) {
		rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
		k_usleep(1000);
	}

	return 0;
}
