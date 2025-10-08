// ========== SUBSCRIBER and PUBLISHER WITH MOTORS (with encoders) ===============
#include <Arduino.h>
#include <micro_ros_platformio.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h> // subscriber
#include <nav_msgs/msg/odometry.h> // publisher
#include <std_msgs/msg/int32.h>

#include <rosidl_runtime_c/string_functions.h>

#include "driver/pcnt.h"

// ================= Robot Parameters and Pins =================
#define WHEEL_RADIUS 0.045   // meters
#define TRACK_WIDTH  0.45    // meters
#define MAX_PWM      255
#define MAX_WHEEL_ANGULAR_SPEED 7.0  // rad/s (67 RPM motors)
#define TICKS_PER_REV 9600 // 64 ticks per rev on motor * 150 gear ratio

// Motor pins (adjust to your wiring)
#define left_motor_pwm_pin 13
#define left_motor_dir_pin_1 25
#define left_motor_dir_pin_2 26
#define left_motor_encA 35
#define left_motor_encB 34

#define right_motor_pwm_pin 15
#define right_motor_dir_pin_1 12
#define right_motor_dir_pin_2 14
#define right_motor_encA 32 // main board 39
#define right_motor_encB 33 // main board 26


// ================= micro-ROS & ROS2 Globals =================
#define RXD2 16
#define TXD2 17

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;

// publisher
rcl_publisher_t publisher;
nav_msgs__msg__Odometry odom_msg;
std_msgs__msg__Int32 msg;
rclc_executor_t executor_pub;
rcl_timer_t timer;

// subscriber
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist cmd_vel_msg;
rclc_executor_t executor_sub;

// odometry and ecoder global variables
volatile long left_ticks = 0;
volatile long right_ticks = 0;

float x = 0.0, y = 0.0, theta = 0.0;
float linear = 0.0, angular = 0.0;
long last_left_ticks = 0, last_right_ticks = 0;
unsigned long last_time = 0;

// encoder ISRS
//void IRAM_ATTR leftEncoderISR() { left_ticks++; }
//void IRAM_ATTR rightEncoderISR() { right_ticks++; }

// encoder pcnt (pulse counters)
pcnt_unit_t pcnt_unit_left = PCNT_UNIT_0;  // Left motor
pcnt_unit_t pcnt_unit_right = PCNT_UNIT_1; // Right motor

// ================= Utility Functions =================

// Error loop if init fails
void error_loop() {
  while (1) {
    delay(100);
    Serial.println("Error");
  }
}

// function to read encoder ticks
long getEncoderCount(pcnt_unit_t unit) {
  int16_t count;
  pcnt_get_counter_value(unit, &count); // Read hardware counter
  return (long)count;                  // Cast to long
}

void computeOdometry() {
  unsigned long current_time = millis();
  float dt = (current_time - last_time) / 1000.0;
  last_time = current_time;

  left_ticks = getEncoderCount(pcnt_unit_left);
  right_ticks = getEncoderCount(pcnt_unit_right);

  long delta_left = left_ticks - last_left_ticks;
  long delta_right = right_ticks - last_right_ticks;
  last_left_ticks = left_ticks;
  last_right_ticks = right_ticks;

  float dist_left = (2 * PI * WHEEL_RADIUS * delta_left) / TICKS_PER_REV;
  float dist_right = (2 * PI * WHEEL_RADIUS * delta_right) / TICKS_PER_REV;
  
  // Velocities of each wheel
  float v_left = dist_left / dt;
  float v_right = dist_right / dt;
  
  // Robot linear & angular velocities
  linear = (v_right + v_left) / 2.0;
  angular = (v_right - v_left) / TRACK_WIDTH;

  // pose update
  float dist = (dist_right + dist_left) / 2.0;
  float dtheta = (dist_right - dist_left) / TRACK_WIDTH;

  x += dist * cos(theta + dtheta / 2.0);
  y += dist * sin(theta + dtheta / 2.0);
  theta += dtheta;

  // Normalize theta to [-pi, pi]
  if (theta > PI) theta -= 2 * PI;
  else if (theta < -PI) theta += 2 * PI;
}

// ================= Publisher Callback =================
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  RCLC_UNUSED(last_call_time);

  computeOdometry();

  // --- Fill Odometry Message ---
  uint64_t now_ms = rmw_uros_epoch_millis();
  odom_msg.header.stamp.sec = (int32_t) (now_ms / 1000);
  odom_msg.header.stamp.nanosec = (uint32_t) (now_ms % 1000) * 1000000;

  rosidl_runtime_c__String__assign(&odom_msg.header.frame_id, "odom");
  rosidl_runtime_c__String__assign(&odom_msg.child_frame_id, "base_link");

  odom_msg.pose.pose.position.x = x;
  odom_msg.pose.pose.position.y = y;
  odom_msg.pose.pose.position.z = 0.0;

  float qz = sin(theta / 2.0);
  float qw = cos(theta / 2.0);
  odom_msg.pose.pose.orientation.z = qz;
  odom_msg.pose.pose.orientation.w = qw;

  odom_msg.twist.twist.linear.x = linear;
  odom_msg.twist.twist.angular.z = angular;

  // Publish
  RCCHECK(rcl_publish(&publisher, &odom_msg, NULL));

  // if (timer != NULL) {
  //   RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  //   msg.data++;
  // }
}

// setupEncoder for pcnt
void setupEncoder(pcnt_unit_t unit, int pinA, int pinB) {
  pcnt_config_t pcnt_config;
  
  pcnt_config.pulse_gpio_num = pinA;        // A channel
  pcnt_config.ctrl_gpio_num = pinB;         // B channel (direction)
  pcnt_config.channel = PCNT_CHANNEL_0;
  pcnt_config.unit = unit;
  
  pcnt_config.pos_mode = PCNT_COUNT_INC;    // Count up on rising edge of A
  pcnt_config.neg_mode = PCNT_COUNT_DEC;    // Count down on falling edge of A
  pcnt_config.lctrl_mode = PCNT_MODE_REVERSE; // Reverse if B HIGH
  pcnt_config.hctrl_mode = PCNT_MODE_KEEP;    // Keep direction if B LOW

  pcnt_config.counter_h_lim = 32767;        // max counter
  pcnt_config.counter_l_lim = -32768;       // min counter

  pcnt_unit_config(&pcnt_config);           // Apply configuration
  pcnt_counter_clear(unit);                 // Reset counter
  pcnt_counter_resume(unit);                // Start counting
}

// Map wheel angular speed (rad/s) to PWM
int wheel_speed_to_pwm(float omega) {
  if (omega > MAX_WHEEL_ANGULAR_SPEED) omega = MAX_WHEEL_ANGULAR_SPEED;
  if (omega < -MAX_WHEEL_ANGULAR_SPEED) omega = -MAX_WHEEL_ANGULAR_SPEED;
  return (int)(abs(omega) / MAX_WHEEL_ANGULAR_SPEED * MAX_PWM);
}

// Send PWM and direction to motors
void set_motor_pwm(int left_pwm, int right_pwm, int left_dir, int right_dir) {
  Serial.println("In set_motor_pwm");
  digitalWrite(left_motor_dir_pin_1, left_dir);
  digitalWrite(left_motor_dir_pin_2, (!left_dir));

  digitalWrite(right_motor_dir_pin_1, right_dir);
  digitalWrite(right_motor_dir_pin_2, (!right_dir));

  analogWrite(left_motor_pwm_pin, left_pwm);
  analogWrite(right_motor_pwm_pin, right_pwm);
}

// ================= Subscriber Callback =================
void cmd_vel_callback(const void * msgin)
{
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  float linear_x  = msg->linear.x;   // m/s
  float angular_z = msg->angular.z;  // rad/s

  Serial.printf("Received cmd_vel -> linear.x: %.2f, angular.z: %.2f\n", linear_x, angular_z);

  // Differential drive formulas
  float v_left  = linear_x - (TRACK_WIDTH / 2.0) * angular_z;
  float v_right = linear_x + (TRACK_WIDTH / 2.0) * angular_z;

  // Convert to angular velocity
  float omega_left  = v_left / WHEEL_RADIUS;
  float omega_right = v_right / WHEEL_RADIUS;

  // Map to PWM
  int left_pwm  = wheel_speed_to_pwm(omega_left);
  int right_pwm = wheel_speed_to_pwm(omega_right);

  // Determine motor direction
  int left_dir  = (omega_left >= 0) ? HIGH : LOW;
  int right_dir = (omega_right >= 0) ? HIGH : LOW;

  // Send to motors
  set_motor_pwm(left_pwm, right_pwm, left_dir, right_dir);

  Serial.printf("Wheel omega -> left: %.2f, right: %.2f rad/s\n", omega_left, omega_right);
  Serial.printf("PWM -> left: %d, right: %d\n", left_pwm, right_pwm);
}

// ================= Setup & Loop =================
void setup() {
  // Setup motor pins
  pinMode(left_motor_pwm_pin, OUTPUT);
  pinMode(left_motor_dir_pin_1, OUTPUT);
  pinMode(left_motor_dir_pin_2, OUTPUT);
  pinMode(right_motor_pwm_pin, OUTPUT);
  pinMode(right_motor_dir_pin_1, OUTPUT);
  pinMode(right_motor_dir_pin_2, OUTPUT);

  // set up encoder pins
  pinMode(left_motor_encA, INPUT_PULLUP);
  pinMode(left_motor_encB, INPUT_PULLUP);
  pinMode(right_motor_encA, INPUT_PULLUP);
  pinMode(right_motor_encB, INPUT_PULLUP);
  // attachInterrupt(digitalPinToInterrupt(left_motor_encA), leftEncoderISR, RISING);
  // attachInterrupt(digitalPinToInterrupt(right_motor_encA), rightEncoderISR, RISING);

  // Initialize left and right encoders
  setupEncoder(pcnt_unit_left, left_motor_encA, left_motor_encB);
  setupEncoder(pcnt_unit_right, right_motor_encA, right_motor_encB);

  // Start serial for debugging
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  delay(2000);

  Serial.println("before micro-ros");

  // Set micro-ROS transport (UART via Serial2)
  set_microros_serial_transports(Serial2);

  Serial.println("set micro-ros transport");

  allocator = rcl_get_default_allocator();

  // Initialize micro-ROS support
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
    error_loop();
  }

  Serial.println("Initialize micro-ROS support");

  // Create node
  if (rclc_node_init_default(&node, "esp32_node", "", &support) != RCL_RET_OK) {
    error_loop();
  }

  Serial.println("Create node");

  // Create subscriber to /cmd_vel
  if (rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel") != RCL_RET_OK) {
    error_loop();
  }

  Serial.println("Create Subscriber");

  // create publisher 
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "micro_ros_odom_publisher"));

    Serial.println("create publisher");

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

    Serial.println("create timer");

  // Create subscriber executor and add subscriber
  executor_sub = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));

  Serial.println("created subscriber");

  // create publisher executor and add timer
  RCCHECK(rclc_executor_init(&executor_pub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_pub, &timer));

  Serial.println("created publisher");

  msg.data = 0;
  Serial.println("ESP32 Differential Drive Node Ready!");
  delay(10);
}

void loop() {
  delay(100);
  // Run executor periodically
  RCCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(100)));
  RCCHECK(rclc_executor_spin_some(&executor_pub, RCL_MS_TO_NS(100)));
}
