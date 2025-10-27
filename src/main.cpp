// ========== SUBSCRIBER and PUBLISHER WITH MOTORS (with encoders) ===============
#include <Arduino.h>
#include <micro_ros_platformio.h>

// micro-ros
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <mutex>

#include <geometry_msgs/msg/twist.h> // subscriber
#include <nav_msgs/msg/odometry.h> // publisher for wheel encoders
#include <sensor_msgs/msg/imu.h> // publisher for imu
#include <sensor_msgs/msg/joint_state.h> // publisher for joint states
//#include <geometry_msgs/msg/transform_stamped.h> // publisher for tf
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

// ultrasonic sensors


// ================= Robot Parameters and Pins =================
#define WHEEL_RADIUS 0.045   // meters
#define TRACK_WIDTH 0.45   // meters
#define MAX_PWM 255
#define MIN_PWM 45 // could be as low as 40 tbh
#define MAX_WHEEL_ANGULAR_SPEED 15.71  // rad/s (2 * pi * 150RPM) / 60s
#define DRIVE_GEAR_REDUCTION 1.0 // no external gears
#define TICKS_PER_REV 64 // from motor spec sheet
#define ENCODER_COUNTS_PER_REV 4480 // 64 ticks per rev on motor * 70 gear ratio
#define DISTANCE_PER_COUNT (PI * 2 * WHEEL_RADIUS) / (ENCODER_COUNTS_PER_REV) // in meters

// Motor pins (adjust to your wiring)
#define left_motor_pwm_pin 13
#define left_motor_dir_pin_1 25
#define left_motor_dir_pin_2 26
#define left_motor_encA 33
#define left_motor_encB 32

#define right_motor_pwm_pin 15
#define right_motor_dir_pin_1 12
#define right_motor_dir_pin_2 14
#define right_motor_encA 35 // main board 39 // swapping these pins to see if it fixes odometry
#define right_motor_encB 34 // main board 26

// ======================== UART Pins ============================
#define RPi_RX 16
#define RPi_TX 17

#define RFID_RX 4
#define RFID_TX 27

// IMU pins
// SCL 22
// SDA 21


// ====== Globals ======
volatile bool obstacleDetected = false;
hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

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
rcl_publisher_t publisher_odom_tf;
nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
sensor_msgs__msg__JointState joint_state_msg;
// geometry_msgs__msg__TransformStamped tf_msg;

std_msgs__msg__Int32 msg;
rclc_executor_t executor_pub_odom;
rclc_executor_t executor_pub_imu;
rclc_executor_t executor_pub_joint_state;
rclc_executor_t executor_odom_tf;
rcl_timer_t odom_timer;
rcl_timer_t imu_timer;
rcl_timer_t joint_state_timer;

// subscriber
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist cmd_vel_msg;
rclc_executor_t executor_sub;
geometry_msgs__msg__Twist last_cmd_vel_msg;
std::mutex cmd_vel_mutex;
unsigned long last_cmd_time = 0;

// publisher joint state globals
rosidl_runtime_c__String name_data[2];
double position_data[2];
double velocity_data[2];

// odometry and ecoder global variables
// volatile long left_ticks = 0;
// volatile long right_ticks = 0;

// float x = 0.0, y = 0.0, theta = 0.0;
// float linear = 0.0, angular = 0.0;
long last_left_ticks = 0, last_right_ticks = 0;
unsigned long last_time = 0;

//final odometric datas
double x;
double y;
double th;
double v_left; // measured left motor speed
double v_right; // measured right motor speed
double vth; // angular velocity of robot
double deltaLeft; // no of ticks in left encoder since last update
double deltaRight; // no of ticks in right encoder since last update
double dt;
double delta_distance; // distance moved by robot since last update
double delta_th; // corresponging change in heading
double delta_x ; // corresponding change in x direction
double delta_y; // corresponding change in y direction

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
  return (long)count;                   // Cast to long
}

void computeOdometry() {
  unsigned long current_time = millis();
  dt = (current_time - last_time) / 1000.0;
  last_time = current_time;

  double left_ticks = getEncoderCount(pcnt_unit_left);
  double right_ticks = getEncoderCount(pcnt_unit_right);
  Serial.print("odom left ticks: ");   Serial.println(left_ticks);
  Serial.print("odom right ticks: ");  Serial.println(right_ticks);

  deltaLeft = left_ticks - last_left_ticks;
  deltaRight = right_ticks - last_right_ticks;
  last_left_ticks = left_ticks;
  last_right_ticks = right_ticks;
  Serial.print("odom deltaLeft: ");   Serial.println(deltaLeft);
  Serial.print("odom deltaRight: ");  Serial.println(deltaRight);

  v_left = deltaLeft * DISTANCE_PER_COUNT/dt;    // measured speed = m/s 
  v_right = deltaRight * DISTANCE_PER_COUNT/dt;  // measured speed = m/s

  Serial.printf("v_left: %f, v_right: %f\n", v_left, v_right);

  delta_distance = 0.5f * (double)(deltaLeft + deltaRight) * DISTANCE_PER_COUNT;
  Serial.print("odom delta_distance: ");  Serial.println(delta_distance);
  
  delta_th = (double)(deltaRight-deltaLeft)* DISTANCE_PER_COUNT/TRACK_WIDTH;
  Serial.print("odom delta_th: ");  Serial.println(delta_th);

  delta_x = delta_distance*(double)cos(th);
  delta_y = delta_distance*(double)sin(th);
  Serial.print("odom delta_x: ");  Serial.println(delta_x);
  Serial.print("odom delta_y: ");  Serial.println(delta_y);

  Serial.println("");

  x += delta_x;
  y += delta_y;
  th += delta_th;

  //float dist_left = (2 * PI * WHEEL_RADIUS * deltaLeft) / TICKS_PER_REV;
  // float dist_right = (2 * PI * WHEEL_RADIUS * deltaRight) / TICKS_PER_REV;
  
  // Velocities of each wheel
  //float v_left = dist_left / dt;
  //float v_right = dist_right / dt;
  
  // Robot linear & angular velocities
  // linear = (v_right + v_left) / 2.0;
  // angular = (v_right - v_left) / TRACK_WIDTH;

  // // pose update
  // float dist = (dist_right + dist_left) / 2.0;
  // float dtheta = (dist_right - dist_left) / TRACK_WIDTH;

  // x += dist * cos(theta + dtheta / 2.0);
  // y += dist * sin(theta + dtheta / 2.0);
  // theta += dtheta;

  // // Normalize theta to [-pi, pi]
  // if (theta > PI) theta -= 2 * PI;
  // else if (theta < -PI) theta += 2 * PI;
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

  // convert euler th to quaternions
  float qz = sin(th / 2.0);
  float qw = cos(th / 2.0);

  odom_msg.pose.pose.orientation.x = 0;
  odom_msg.pose.pose.orientation.y = 0;
  odom_msg.pose.pose.orientation.z = qz;
  odom_msg.pose.pose.orientation.w = qw;

  odom_msg.twist.twist.linear.x = delta_x/dt;
  odom_msg.twist.twist.angular.z = delta_th/dt;

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

// Map wheel angular speed (rad/s) to PWM - returns PWM
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
  // Serial.print("left ticks: ");   Serial.println(left_ticks);
  // Serial.print("right ticks: ");   Serial.println(right_ticks);

  double left_angle = (left_ticks / TICKS_PER_REV) * 2.0 * M_PI;
  double right_angle = (right_ticks / TICKS_PER_REV) * 2.0 * M_PI;
  // Serial.print("left angle: ");   Serial.println(left_angle);

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

  // Serial.printf("Received cmd_vel -> linear.x: %.2f, angular.z: %.2f\n", linear_x, angular_z);

  // Differential drive formulas - calculate the requested v_left and v_right
  float req_v_left  = linear_x - (TRACK_WIDTH / 2.0) * angular_z;
  float req_v_right = linear_x + (TRACK_WIDTH / 2.0) * angular_z;

  Serial.printf("angular z: %f\n", angular_z);
  Serial.printf("linear x: %f\n", linear_x);

  // Convert to angular velocity
  float omega_left  = req_v_left / WHEEL_RADIUS;
  float omega_right = req_v_right / WHEEL_RADIUS;

  // Map to PWM - simple diff - original left and right pwm requests
  int left_pwm  = wheel_speed_to_pwm(omega_left);
  int right_pwm = wheel_speed_to_pwm(omega_right);

  Serial.printf("pwm req: left %lf, right %f\n", left_pwm, right_pwm);

  // Determine motor direction - simple diff
  int left_dir  = (omega_left >= 0) ? HIGH : LOW;
  int right_dir = (omega_right >= 0) ? HIGH : LOW;

 if (fabs(angular_z) < 0.1 && fabs(linear_x) > 0.05) {
    Serial.println("inside straight correction");
    // average different in actual wheel speeds for 3 cycles
    double angular_vel_diff = v_left - v_right;
    Serial.printf("angular_vel_diff: %lf\n", angular_vel_diff);
    static double prev_diff = 0;
    static double prev_diff2 = 0;
    Serial.printf("prev_diff: %lf, prev_diff2: %lf\n", prev_diff, prev_diff2);
    double avg_angular_diff = (prev_diff + prev_diff2 + angular_vel_diff) / 3;
    Serial.printf("avg_angular_diff: %lf\n", avg_angular_diff);
    prev_diff2 = prev_diff;
    prev_diff = angular_vel_diff;

    // apply correction to each wheel to try and go straight
    left_pwm -= (int) (avg_angular_diff * 125); // 125 rad --> 12.5 pwm
    right_pwm += (int) (avg_angular_diff * 125); 
  }

  if (abs(left_pwm) <= MIN_PWM) left_pwm = 0;
  if (abs(right_pwm) <= MIN_PWM) right_pwm = 0;

  // Send to motors
  set_motor_pwm(left_pwm, right_pwm, left_dir, right_dir);

  // Serial.printf("Wheel omega -> left: %.2f, right: %.2f rad/s\n", omega_left, omega_right);
  Serial.printf("Final PWM -- left: %d, right: %d\n", left_pwm, right_pwm);
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
  // Serial.println("RFID is now initialized, configured, and ready to read!");
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
  // Serial.println("Adafruit MPU6050 test!");

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("IMU Found!");

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

  // Start serial for debugging
  Serial.begin(115200);
  delay(100);
  Serial.println("PARCEL is booting up.");
  Serial.println("Motor Pins Initialized.");

  // Initialize left and right encoders
  setupEncoder(pcnt_unit_left, left_motor_encA, left_motor_encB);
  setupEncoder(pcnt_unit_right, right_motor_encA, right_motor_encB);

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

  allocator = rcl_get_default_allocator();

  // Initialize micro-ROS support
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
    error_loop();
  }

  // Create node
  if (rclc_node_init_default(&node, "esp32_node", "", &support) != RCL_RET_OK) {
    error_loop();
  }

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

  // // create odom tf publisher 
  // RCCHECK(rclc_publisher_init_default(
  //   &publisher_odom_tf,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(tf2_msgs, msg, TFMessage),
  //   "micro_ros_joint_state_publisher"));

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
  Serial.println("Connection to Raspberry Pi Complete!");
  delay(10);
  Serial.println("System Finished Initializing.");
  Serial.println("PARCEL is waiting for a package.");
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

  // imuStats();

  // Do other stuff here
  // delay(10);

  uint8_t success;
  uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
  uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)

  uint8_t data[16];
  uint8_t keya[6] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

  static unsigned long lastTrigger = 0;
  unsigned long now = millis();
  static unsigned long lastPrint = 0;


  // State Machine
  switch(currentState) {
    case SystemState::idle: 
      currentState = SystemState::normalMove;
      break;

    case SystemState::rfid:
      // Serial.println("PARCEL is waiting for a package.");
        
      // Wait for an ISO14443A type cards (Mifare, etc.).  When one is found
      // 'uid' will be populated with the UID, and uidLength will indicate
      // if the uid is 4 bytes (Mifare Classic) or 7 bytes (Mifare Ultralight)
      success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength);

        if (success) {
          if (uidLength == 4)
          {
            // Start with block 4 (the first block of sector 1) since sector 0
            // contains the manufacturer data and it's probably better just
            // to leave it alone unless you know what you're doing
            success = nfc.mifareclassic_AuthenticateBlock(uid, uidLength, 4, 0, keya);
          
            if (success)
            {
              // Serial.println("Sector 1 (Blocks 4..7) has been authenticated");
              // uint8_t data[16];
          
              // If you want to write something to block 4 to test with, uncomment
              // the following line and this text should be read back in a minute
              //uint8_t data[16] = { 'P', 'a', 'c', 'k', 'a', 'g', 'e', ' ', 'A', ' ', ' ', ' ', ' ', ' ', ' ', ' '};
              
              // success = nfc.mifareclassic_WriteDataBlock (4, data);

              // Try to read the contents of block 4
              success = nfc.mifareclassic_ReadDataBlock(4, data);
          
              if (success)
              {
                // Data seems to have been read ... spit it out
                Serial.println("Reading Package RFID Tag:");
                nfc.PrintHexChar(data, 16);
                Serial.println("");
            
                // Move to normalMove
                Serial.println("Making route to location.");
                currentState = SystemState::normalMove;
              }
              else
              {
                Serial.println("Ooops ... unable to read the requested block.  Try another key?");
              }
            } else {
              Serial.println("Ooops ... authentication failed: Try another key?");
            }
          }

      break;

    case SystemState::normalMove:
      
      break;

    case SystemState::obstacle: 
        Serial.println("Obstacle detected within range! Set motors to stop!");
        set_motor_pwm(0, 0, LOW, LOW); // stop motors
        currentState = SystemState::normalMove;
      break;

    case SystemState::rerouting: 
      break;

    case SystemState::removePayload: 
      break;
  }
  }
}
