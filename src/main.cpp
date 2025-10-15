// ========== SUBSCRIBER and PUBLISHER WITH MOTORS (with encoders) ===============
#include <Arduino.h>
#include <micro_ros_platformio.h>

// micro-ros
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <geometry_msgs/msg/twist.h> // subscriber
#include <nav_msgs/msg/odometry.h> // publisher for wheel encoders
#include <sensor_msgs/msg/imu.h> // publisher for imu
#include <sensor_msgs/msg/joint_state.h> // publisher for joint states
#include <std_msgs/msg/int32.h>

#include <rosidl_runtime_c/string_functions.h>

// pulse counter for motor encoders
#include "driver/pcnt.h"

// RIFD
#include <PN532_HSU.h>
#include <PN532.h>

// IMU
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

// ================= Robot Parameters and Pins =================
#define WHEEL_RADIUS 0.045   // meters
#define TRACK_WIDTH  0.45    // meters
#define MAX_PWM      255
#define MAX_WHEEL_ANGULAR_SPEED 7.0  // rad/s (67 RPM motors)
#define TICKS_PER_REV 9600 // 64 ticks per rev on motor * 150 gear ratio

// Motor pins (adjust to your wiring)
#define left_motor_pwm_pin 13
#define left_motor_dir_pin_1 26
#define left_motor_dir_pin_2 25
#define left_motor_encA 35
#define left_motor_encB 34

#define right_motor_pwm_pin 15
#define right_motor_dir_pin_1 14
#define right_motor_dir_pin_2 12
#define right_motor_encA 32 // main board 39
#define right_motor_encB 33 // main board 26

// ======================== UART Pins ============================
#define RPi_RX 16
#define RPi_TX 17

#define RFID_RX 4
#define RFID_TX 27

// IMU pins
// SCL 22
// SDA 21

// ================= micro-ROS & ROS2 Globals =====================
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

rcl_node_t node;
rclc_support_t support;
rcl_allocator_t allocator;

// publisher
rcl_publisher_t publisher_odom;
rcl_publisher_t publisher_imu;
rcl_publisher_t publisher_joint_state;
nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__JointState joint_state_msg;
std_msgs__msg__Int32 msg;
rclc_executor_t executor_pub_odom;
rclc_executor_t executor_pub_imu;
rclc_executor_t executor_pub_joint_state;
rcl_timer_t odom_timer;
rcl_timer_t imu_timer;
rcl_timer_t joint_state_timer;

// subscriber
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist cmd_vel_msg;
rclc_executor_t executor_sub;

// publisher joint state globals
rosidl_runtime_c__String name_data[2];
double position_data[2];
double velocity_data[2];

// odometry and ecoder global variables
// volatile long left_ticks = 0;
// volatile long right_ticks = 0;

float x = 0.0, y = 0.0, theta = 0.0;
float linear = 0.0, angular = 0.0;
long last_left_ticks = 0, last_right_ticks = 0;
unsigned long last_time = 0;

// encoder ISRS
// void IRAM_ATTR leftEncoderISR() { left_ticks++; }
// void IRAM_ATTR rightEncoderISR() { right_ticks++; }

// encoder pcnt (pulse counters)
pcnt_unit_t pcnt_unit_left = PCNT_UNIT_0;  // Left motor
pcnt_unit_t pcnt_unit_right = PCNT_UNIT_1; // Right motor

// RFID
PN532_HSU pn532hsu(Serial1);
PN532 nfc(pn532hsu);

// IMU
Adafruit_MPU6050 mpu;

// ================= Utility Functions for Micro-ROS =================

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

  double left_ticks = getEncoderCount(pcnt_unit_left);
  double right_ticks = getEncoderCount(pcnt_unit_right);
  Serial.print("odom left ticks: ");   Serial.println(left_ticks);
  Serial.print("odom right ticks: ");   Serial.println(right_ticks);

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

// ================= ODOM Publisher Callback =================
void odom_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
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
  RCCHECK(rcl_publish(&publisher_odom, &odom_msg, NULL));

  // if (timer != NULL) {
  //   RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  //   msg.data++;
  // }
}

// ================= IMU Publisher Callback =================
void imu_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);

  // --- header ---
  uint64_t now_ms = rmw_uros_epoch_millis();
  imu_msg.header.stamp.sec = now_ms / 1000;
  imu_msg.header.stamp.nanosec = (now_ms % 1000) * 1000000;
  rosidl_runtime_c__String__assign(&imu_msg.header.frame_id, "imu");

  // --- Get new sensor events with the readings ---
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  // --- fill angular velocity ---
  imu_msg.angular_velocity.x = g.gyro.x;
  imu_msg.angular_velocity.y = g.gyro.y;
  imu_msg.angular_velocity.z = g.gyro.z;

  // --- fill linear acceleration ---
  imu_msg.linear_acceleration.x = a.acceleration.x;
  imu_msg.linear_acceleration.y = a.acceleration.y;
  imu_msg.linear_acceleration.z = a.acceleration.z;

  // --- Publish ---
  RCCHECK(rcl_publish(&publisher_imu, &imu_msg, NULL));

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

// ================= Joint State Publisher Callback =================
void joint_state_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);

  uint64_t now_ms = rmw_uros_epoch_millis();
  joint_state_msg.header.stamp.sec = (int32_t) (now_ms / 1000);
  joint_state_msg.header.stamp.nanosec = (uint32_t) (now_ms % 1000) * 1000000;

  rosidl_runtime_c__String__assign(&joint_state_msg.header.frame_id, "joint state");

  joint_state_msg.name.capacity = 2;
  joint_state_msg.name.size = 2;
  joint_state_msg.name.data = name_data;
  rosidl_runtime_c__String__assign(&joint_state_msg.name.data[0], "left_wheel_joint");
  rosidl_runtime_c__String__assign(&joint_state_msg.name.data[1], "right_wheel_joint");

  joint_state_msg.velocity.capacity = 2;
  joint_state_msg.velocity.size = 2;
  joint_state_msg.velocity.data = velocity_data;

  double left_ticks = getEncoderCount(pcnt_unit_left);
  double right_ticks = getEncoderCount(pcnt_unit_right);
  Serial.print("left ticks: ");   Serial.println(left_ticks);
  Serial.print("right ticks: ");   Serial.println(right_ticks);

  double left_angle = (left_ticks / TICKS_PER_REV) * 2.0 * M_PI;
  double right_angle = (right_ticks / TICKS_PER_REV) * 2.0 * M_PI;
  Serial.print("left angle: ");   Serial.println(left_angle);

  // Update wheel joint positions
  position_data[0] = left_angle;
  position_data[1] = right_angle;

  joint_state_msg.position.capacity = 2;
  joint_state_msg.position.size = 2;
  joint_state_msg.position.data = position_data;

  // --- Publish ---
  RCCHECK(rcl_publish(&publisher_joint_state, &joint_state_msg, NULL));
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

void rfidInitalize()
{
  nfc.begin();

  delay(2000);

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (! versiondata) {
    Serial.print("Didn't find PN53x board");
    while (1); // halt
  }

  // Got ok data, print it out!
  Serial.print("Found chip PN5"); Serial.println((versiondata>>24) & 0xFF, HEX); 
  Serial.print("Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC); 
  Serial.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);
  
  // configure board to read RFID tags
  nfc.SAMConfig();
  // RFID Will now be ready to read at anytime
  Serial.println("RFID is now initialized, configured, and ready to read!");
}

void imuStats() 
{
  /* Get new sensor events with the readings */
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  /* Print out the values */
  Serial.print("Acceleration X: ");
  Serial.print(a.acceleration.x);
  Serial.print(", Y: ");
  Serial.print(a.acceleration.y);
  Serial.print(", Z: ");
  Serial.print(a.acceleration.z);
  Serial.println(" m/s^2");

  Serial.print("Rotation X: ");
  Serial.print(g.gyro.x);
  Serial.print(", Y: ");
  Serial.print(g.gyro.y);
  Serial.print(", Z: ");
  Serial.print(g.gyro.z);
  Serial.println(" rad/s");

  Serial.print("Temperature: ");
  Serial.print(temp.temperature);
  Serial.println(" degC");

  Serial.println("");
  delay(500);
}

void imuInitialize()
{
  Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  Serial.print("Accelerometer range set to: ");
  switch (mpu.getAccelerometerRange()) {
  case MPU6050_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case MPU6050_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case MPU6050_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case MPU6050_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  Serial.print("Gyro range set to: ");
  switch (mpu.getGyroRange()) {
  case MPU6050_RANGE_250_DEG:
    Serial.println("+- 250 deg/s");
    break;
  case MPU6050_RANGE_500_DEG:
    Serial.println("+- 500 deg/s");
    break;
  case MPU6050_RANGE_1000_DEG:
    Serial.println("+- 1000 deg/s");
    break;
  case MPU6050_RANGE_2000_DEG:
    Serial.println("+- 2000 deg/s");
    break;
  }

  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.print("Filter bandwidth set to: ");
  switch (mpu.getFilterBandwidth()) {
  case MPU6050_BAND_260_HZ:
    Serial.println("260 Hz");
    break;
  case MPU6050_BAND_184_HZ:
    Serial.println("184 Hz");
    break;
  case MPU6050_BAND_94_HZ:
    Serial.println("94 Hz");
    break;
  case MPU6050_BAND_44_HZ:
    Serial.println("44 Hz");
    break;
  case MPU6050_BAND_21_HZ:
    Serial.println("21 Hz");
    break;
  case MPU6050_BAND_10_HZ:
    Serial.println("10 Hz");
    break;
  case MPU6050_BAND_5_HZ:
    Serial.println("5 Hz");
    break;
  }

  Serial.println("");
  delay(100);
}

// ======================= Initialization (Setup Function) ========================
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
  
  // Start RPi Serial
  Serial2.begin(115200, SERIAL_8N1, RPi_RX, RPi_TX);
  
  // IMU Initialize
  imuInitialize();
  Serial.println("IMU Initialized!");

  // RFID Initalize
  // rfidInitalize();
  // Serial.println("RFID Initialized!");

  // Set micro-ROS transport (UART via Serial2)
  set_microros_serial_transports(Serial2);

  Serial.println("micro-ros transport");

  allocator = rcl_get_default_allocator();

  // Initialize micro-ROS support
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
    error_loop();
  }

  Serial.println("initialize micro-ros support");

  // Create node
  if (rclc_node_init_default(&node, "esp32_node", "", &support) != RCL_RET_OK) {
    error_loop();
  }

  Serial.println("create micro-ros node");

  // Create subscriber to /cmd_vel
  if (rclc_subscription_init_default(
        &subscriber,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel") != RCL_RET_OK) {
    error_loop();
  }

  // create odom publisher 
  RCCHECK(rclc_publisher_init_default(
    &publisher_odom,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "micro_ros_odom_publisher"));

  // create imu publisher 
  RCCHECK(rclc_publisher_init_default(
    &publisher_imu,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "micro_ros_imu_publisher"));

  // create joint state publisher 
  RCCHECK(rclc_publisher_init_default(
    &publisher_joint_state,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "micro_ros_joint_state_publisher"));

  // create odom timer,
  const unsigned int timer_timeout_odom = 1000;
  RCCHECK(rclc_timer_init_default(
    &odom_timer,
    &support,
    RCL_MS_TO_NS(timer_timeout_odom),
    odom_timer_callback));

  // create imu timer
  const unsigned int timer_timeout_imu = 1000;
  RCCHECK(rclc_timer_init_default(
    &imu_timer,
    &support,
    RCL_MS_TO_NS(timer_timeout_imu),
    imu_timer_callback));

  // create joint state timer
  const unsigned int timer_timeout_joint_state = 1000;
  RCCHECK(rclc_timer_init_default(
    &joint_state_timer,
    &support,
    RCL_MS_TO_NS(timer_timeout_joint_state),
    joint_state_timer_callback));

  // Create subscriber executor and add subscriber
  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));

  // create odom publisher executor and add timer
  RCCHECK(rclc_executor_init(&executor_pub_odom, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_pub_odom, &odom_timer));

  // create imu publisher executor and add timer
  RCCHECK(rclc_executor_init(&executor_pub_imu, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_pub_imu, &imu_timer));

  // create joint state publisher executor and add timer
  RCCHECK(rclc_executor_init(&executor_pub_joint_state, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_pub_joint_state, &joint_state_timer));


  msg.data = 0;
  Serial.println("Micro-ROS Connection Complete!");
  delay(10);
  Serial.println("System Finished Initialized.");
}

// Define state machine
enum class SystemState {
        idle,
        rfid,
        normalMove,
        obstacle,
        rerouting,
        removePayload
    };

SystemState currentState = SystemState::idle;

// ======================= State Machine (loop Function) ========================
void loop() {
  delay(100);
  // Run executor periodically
  RCCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(100)));
  RCCHECK(rclc_executor_spin_some(&executor_pub_odom, RCL_MS_TO_NS(100)));
  RCCHECK(rclc_executor_spin_some(&executor_pub_imu, RCL_MS_TO_NS(100)));
  RCCHECK(rclc_executor_spin_some(&executor_pub_joint_state, RCL_MS_TO_NS(100)));

  imuStats();

  /* State Machine

  switch(state) {
    case idle: 
      break;

    case rfid:

      break;

    case normalMove: 
      break;

    case obstacle: 
      break;

    case rerouting: 
      break;

    case removePayload: 
      break;
  }
  

  */
}
