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
// #include <sensor_msgs/msg/joint_state.h> // publisher for joint states
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>
// #include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_msgs/msg/tf_message.h>

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
#include "I2Cdev.h"
#include "MPU6050.h"

// wi-fi 
#include <WiFi.h>
#include <PubSubClient.h>
#include <certs.h>
#include <WiFiClientSecure.h>

// IO Expander
#include "TCA9554.h"

// ================= Robot Parameters and Pins =================
#define WHEEL_RADIUS 0.045   // meters
#define TRACK_WIDTH 0.45   // meters
#define MAX_PWM 255
#define MIN_PWM 45 // could be as low as 40 tbh
#define MAX_WHEEL_ANGULAR_SPEED 15.71  // rad/s (2 * pi * 150RPM) / 60s
#define DRIVE_GEAR_REDUCTION 1.0 // no external gears
#define TICKS_PER_REV 64 // from motor spec sheet
#define ENCODER_COUNTS_PER_REV 2240 // 64 ticks per rev on motor * 70 gear ratio
#define DISTANCE_PER_COUNT (PI * 2 * WHEEL_RADIUS) / (ENCODER_COUNTS_PER_REV) // in meters

// Motor pins
#define left_motor_pwm_pin 13
#define left_motor_dir_pin_1 26
#define left_motor_dir_pin_2 25
#define left_motor_encA 34
#define left_motor_encB 35

#define right_motor_pwm_pin 15
#define right_motor_dir_pin_1 12
#define right_motor_dir_pin_2 14
#define right_motor_encA 39
#define right_motor_encB 36 

// ======================== UART Pins ============================
#define RPi_RX 16
#define RPi_TX 17

#define RFID_RX 4
#define RFID_TX 27

// IMU pins
// SCL 22
// SDA 21

// IO Expander
TCA9554 TCA(0x20);

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
rcl_publisher_t publisher_tf;
nav_msgs__msg__Odometry odom_msg;
sensor_msgs__msg__Imu imu_msg;
tf2_msgs__msg__TFMessage tf_msg;
geometry_msgs__msg__TransformStamped odom_tf;

std_msgs__msg__Int32 msg;
std_msgs__msg__String rfid_location_msg;
rclc_executor_t executor_pub_odom;
rclc_executor_t executor_pub_imu;
// rclc_executor_t executor_odom_tf;
rclc_executor_t executor_cmd_vel;
rcl_timer_t control_timer;
rcl_timer_t odom_timer;
rcl_timer_t imu_timer;
rcl_timer_t tf_timer;

// subscriber
rcl_subscription_t subscriber;
geometry_msgs__msg__Twist cmd_vel_msg;
rclc_executor_t executor_sub;
geometry_msgs__msg__Twist last_cmd_vel_msg;
std::mutex cmd_vel_mutex;
unsigned long last_cmd_time = 0;

// // publisher joint state globals
// rosidl_runtime_c__String name_data[2];
// double position_data[2];
// double velocity_data[2];

// imu stats
int16_t ax, ay, az;
int16_t gx, gy, gz;
volatile int16_t si_ax, si_ay, si_az;
volatile int16_t si_gx, si_gy, si_gz;

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

// encoder pcnt (pulse counters)
pcnt_unit_t pcnt_unit_left = PCNT_UNIT_0;  // Left motor
pcnt_unit_t pcnt_unit_right = PCNT_UNIT_1; // Right motor

// RFID
PN532_HSU pn532hsu(Serial1);
PN532 nfc(pn532hsu);

// IMU
Adafruit_MPU6050 mpu;
MPU6050 accelgyro; // second imu library

// Wi-Fi
static const char * brokerHost = "r38d6e25.ala.us-east-1.emqxsl.com";
static const uint16_t brokerPort = 8883; // TLS
static const String brokerUser = "parcel";
static const String brokerPass = "password";

WiFiClientSecure secureClient;
PubSubClient mqttClient(secureClient);

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

  double odom_covariances[36] = {0.1, 0, 0, 0, 0, 0, 
                                  0, 0.1, 0, 0, 0, 0,
                                  0, 0, 0.1, 0, 0, 0, 
                                  0, 0, 0, 0.1, 0, 0, 
                                  0, 0, 0, 0, 0.1, 0, 
                                  0, 0, 0, 0, 0, 0.1};

  // copy odom covariance matrix into odom_msg cavariance matrix for pose and twist
  std::copy(std::begin(odom_covariances),
  std::end(odom_covariances),
  std::begin(odom_msg.pose.covariance));

  std::copy(std::begin(odom_covariances),
  std::end(odom_covariances),
  std::begin(odom_msg.twist.covariance));

  odom_msg.twist.twist.linear.x = delta_x/dt;
  odom_msg.twist.twist.linear.y = delta_y/dt; // added
  odom_msg.twist.twist.angular.z = delta_th/dt;

  // Publish
  RCCHECK(rcl_publish(&publisher_odom, &odom_msg, NULL));

  // if (timer != NULL) {
  //   RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
  //   msg.data++;
  // }

  // ---------------------------------------------------------
  // Create odom -> base_link transform for TF
  // ---------------------------------------------------------

  // odom_tf.header.stamp = odom_msg.header.stamp;  // same time as odom
  // odom_tf.header.frame_id.data = "odom";
  // odom_tf.child_frame_id.data = "base_link";

  // // Position
  // odom_tf.transform.translation.x = odom_msg.pose.pose.position.x;
  // odom_tf.transform.translation.y = odom_msg.pose.pose.position.y;
  // odom_tf.transform.translation.z = 0.0;

  // // Orientation (same quaternion as odom)
  // odom_tf.transform.rotation = odom_msg.pose.pose.orientation;

  // // Wrap into TFMessage
  // tf_msg.transforms.size = 1;
  // tf_msg.transforms.data = &odom_tf;

  // Publish TF
  // RCCHECK(rcl_publish(&publisher_tf, &tf_msg, NULL));

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
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  
  double si_ax = ((double) ax / 16384) * 9.80665;
  double si_ay = ((double) ay / 16384) * 9.80665;
  double si_az = ((double) az / 16384) * 9.80665;

  double si_gx = ((double) gx / 131);
  double si_gy = ((double) gy / 131);
  double si_gz = ((double) gz / 131);

  Serial.print("si acc: ");
  Serial.print(si_ax); Serial.print("\t"); 
  Serial.print(si_ay); Serial.print("\t");
  Serial.println(si_az);

  Serial.print("si gyro: ");
  Serial.print(si_gx); Serial.print("\t");
  Serial.print(si_gy); Serial.print("\t");
  Serial.println(si_gz); Serial.print("\t");
  Serial.println("");

  // --- fill angular velocity ---
  imu_msg.angular_velocity.x = si_gx;
  imu_msg.angular_velocity.y = si_gy;
  imu_msg.angular_velocity.z = si_gz;

  // --- fill linear acceleration ---
  imu_msg.linear_acceleration.x = si_ax;
  imu_msg.linear_acceleration.y = si_ay;
  imu_msg.linear_acceleration.z = si_az;

    // --- set covariance matrixes ---
  double imu_covariances[9] = {0.05, 0, 0,
                                0, 0.05, 0,
                                0, 0, 0.05};

  imu_msg.orientation_covariance[0] = -1;

  // copy odom covariance matrix into imu_msg cavariance matrix for linear acc and ang vel
  std::copy(std::begin(imu_covariances),
  std::end(imu_covariances),
  std::begin(imu_msg.linear_acceleration_covariance));

  std::copy(std::begin(imu_covariances),
  std::end(imu_covariances),
  std::begin(imu_msg.angular_velocity_covariance));

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

  digitalWrite(left_motor_dir_pin_1, left_dir);
  digitalWrite(left_motor_dir_pin_2, (!left_dir));

  digitalWrite(right_motor_dir_pin_1, right_dir);
  digitalWrite(right_motor_dir_pin_2, (!right_dir));

  analogWrite(left_motor_pwm_pin, left_pwm);
  analogWrite(right_motor_pwm_pin, right_pwm);
}

// Stops both motors by setting pwms to 0
void stop_motors()
{
  set_motor_pwm(0, 0, LOW, LOW);
}

// // ================= Joint State Publisher Callback =================
// void joint_state_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
// {  
//   RCLC_UNUSED(last_call_time);

//   uint64_t now_ms = rmw_uros_epoch_millis();
//   joint_state_msg.header.stamp.sec = (int32_t) (now_ms / 1000);
//   joint_state_msg.header.stamp.nanosec = (uint32_t) (now_ms % 1000) * 1000000;

//   rosidl_runtime_c__String__assign(&joint_state_msg.header.frame_id, "joint state");

//   joint_state_msg.name.capacity = 2;
//   joint_state_msg.name.size = 2;
//   joint_state_msg.name.data = name_data;
//   rosidl_runtime_c__String__assign(&joint_state_msg.name.data[0], "left_wheel_joint");
//   rosidl_runtime_c__String__assign(&joint_state_msg.name.data[1], "right_wheel_joint");

//   joint_state_msg.velocity.capacity = 2;
//   joint_state_msg.velocity.size = 2;
//   joint_state_msg.velocity.data = velocity_data;

//   double left_ticks = getEncoderCount(pcnt_unit_left);
//   double right_ticks = getEncoderCount(pcnt_unit_right);
//   // Serial.print("left ticks: ");   Serial.println(left_ticks);
//   // Serial.print("right ticks: ");   Serial.println(right_ticks);

//   double left_angle = (left_ticks / TICKS_PER_REV) * 2.0 * M_PI;
//   double right_angle = (right_ticks / TICKS_PER_REV) * 2.0 * M_PI;
//   // Serial.print("left angle: ");   Serial.println(left_angle);

//   // Update wheel joint positions
//   position_data[0] = left_angle;
//   position_data[1] = right_angle;

//   joint_state_msg.position.capacity = 2;
//   joint_state_msg.position.size = 2;
//   joint_state_msg.position.data = position_data;

//   // --- Publish ---
//   RCCHECK(rcl_publish(&publisher_joint_state, &joint_state_msg, NULL));
// }

// ================= Subscriber Callback =================
void cmd_vel_callback(const void * msgin)
{
    const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
    
    std::lock_guard<std::mutex> lock(cmd_vel_mutex);
    last_cmd_vel_msg = *msg;
    last_cmd_time = millis(); // timestamp for timeout
}

void control_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  geometry_msgs__msg__Twist cmd_copy;
  {
      std::lock_guard<std::mutex> lock(cmd_vel_mutex);
      cmd_copy = last_cmd_vel_msg;
  }

  // if (millis() - last_cmd_time > 60000) { // 60 seconds
      //   stop_motors();
      // return;
  // }

  float linear_x  = cmd_copy.linear.x;   // m/s
  float angular_z = cmd_copy.angular.z;  // rad/s

  // Serial.printf("Received cmd_vel -> linear.x: %.2f, angular.z: %.2f\n", linear_x, angular_z);

  // Differential drive formulas - calculate the requested v_left and v_right
  float req_v_left  = linear_x - (TRACK_WIDTH / 2.0) * angular_z;
  float req_v_right = linear_x + (TRACK_WIDTH / 2.0) * angular_z;

  Serial.printf("angular z: %f\n", angular_z);
  Serial.printf("linear x: %f\n", linear_x);

  // Convert to angular velocity
  float omega_left  = req_v_left / WHEEL_RADIUS;
  float omega_right = req_v_right / WHEEL_RADIUS;

  // // Map to PWM - simple diff - original left and right pwm requests
  double left_pwm  = wheel_speed_to_pwm(omega_left);
  double right_pwm = wheel_speed_to_pwm(omega_right);

  Serial.printf("pwm req: left %lf, right %f\n", req_v_left, req_v_right);

  // Determine motor direction - simple diff
  int left_dir  = (omega_left >= 0) ? HIGH : LOW;
  int right_dir = (omega_right >= 0) ? HIGH : LOW;

  // Compute per-wheel error
  float left_error  = req_v_left - v_left; // move dec over
  float right_error = req_v_right - v_right; // move dec over

  Serial.printf("left_error: %f right_error: %f\n", left_error, right_error);

  // Convert error to PWM correction
  // float drift_gain = 120; // tune this, converted v -> PWM
  // Serial.printf("add adjustment for left error %d\n", (int)(left_error * drift_gain));
  // Serial.printf("add adjustment for right error %d\n", (int)(right_error * drift_gain));
  // Serial.printf("left pwm: %d", wheel_speed_to_pwm(req_v_left / WHEEL_RADIUS));
  // Serial.printf("right pwm: %d", wheel_speed_to_pwm(req_v_right / WHEEL_RADIUS));

  // convert to pwm
  // Apply correction in PWM units
  // int left_pwm  = wheel_speed_to_pwm(req_v_left / WHEEL_RADIUS)  + (left_error * drift_gain);
  // int right_pwm = wheel_speed_to_pwm(req_v_right / WHEEL_RADIUS) + (right_error * drift_gain);

  // deadband if too slow of pwm
  if (abs(left_pwm) <= MIN_PWM) {
    left_pwm = 0;
    left_dir = LOW;
  }

  if (abs(right_pwm) <= MIN_PWM) {
    right_pwm = 0;
    right_dir = LOW;
  }

//  if (fabs(angular_z) < 0.1 && fabs(linear_x) > 0.05) {
//     Serial.println("inside straight correction");
//     // average different in actual wheel speeds for 3 cycles
//     double angular_vel_diff = v_left - v_right;
//     Serial.printf("angular_vel_diff: %lf\n", angular_vel_diff);
//     static double prev_diff = 0;
//     static double prev_diff2 = 0;
//     Serial.printf("prev_diff: %lf, prev_diff2: %lf\n", prev_diff, prev_diff2);
//     double avg_angular_diff = (prev_diff + prev_diff2 + angular_vel_diff) / 3;
//     Serial.printf("avg_angular_diff: %lf\n", avg_angular_diff);
//     prev_diff2 = prev_diff;
//     prev_diff = angular_vel_diff;

//     // apply correction to each wheel to try and go straight
//     left_pwm -= (int) (avg_angular_diff * 125); // 125 rad --> 12.5 pwm
//     right_pwm += (int) (avg_angular_diff * 125); 
//   }

//   if (abs(left_pwm) <= MIN_PWM) left_pwm = 0;
//   if (abs(right_pwm) <= MIN_PWM) right_pwm = 0;

  // Send to motors
  // Serial.printf("left_pwm * 1.1 : %d", (int)(left_pwm*1.1));
  
  // left_pwm *= 1.1; // manual bias the left wheel
  
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

void imuInitialize()
{
  Wire.begin();
  accelgyro.initialize();

  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  // set offsets for calibration
  accelgyro.setXAccelOffset(-1414.00000);
  accelgyro.setYAccelOffset(1138.00000);
  accelgyro.setZAccelOffset(1859.00000);
  accelgyro.setXGyroOffset(43.00000);
  accelgyro.setYGyroOffset(-3.00000);
  accelgyro.setZGyroOffset(49.00000);
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

  // Serial.println("before wifi");
  // Wi-Fi
  // secureClient.setCACert(EMQX_ROOT_CA); // gives certification to connect to online broker
  
  // WiFi.begin(brokerUser, brokerPass);

  // while (WiFi.status() != WL_CONNECTED) { delay(250); } // waiting till connected

  // connect MQTT server
  // mqttClient.setServer(brokerHost, brokerPort);

  // Serial.println("after wifi");

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

  // xSerial.println("before tf publisher");
  // create tf publisher
  // RCCHECK(rclc_publisher_init_default(
  //   &publisher_tf,
  //   &node,
  //   ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, TFMessage),
  //   "tf_custom"));

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

  // cmd_control timer
  const unsigned int timer_timeout_cmd_control = 1000;
  RCCHECK(rclc_timer_init_default(
    &control_timer,
    &support,                                 // your rclc support struct
    RCL_MS_TO_NS(timer_timeout_cmd_control),  // 100 Hz
    control_timer_callback));                 // timer callback

  // Create subscriber executor and add subscriber
  RCCHECK(rclc_executor_init(&executor_sub, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_sub, &control_timer));
  RCCHECK(rclc_executor_add_subscription(&executor_sub, &subscriber, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA));

  // create odom publisher executor and add timer
  RCCHECK(rclc_executor_init(&executor_pub_odom, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_pub_odom, &odom_timer));

  // create imu publisher executor and add timer
  RCCHECK(rclc_executor_init(&executor_pub_imu, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor_pub_imu, &imu_timer));

  msg.data = 0;
  
  // Turn on LED with IO Expander
  Wire.begin();
  TCA.begin();

  Wire.setClock(50000);
  TCA.pinMode1(0, OUTPUT);
  TCA.write1(0, HIGH);

  Serial.println("\nConnection to Raspberry Pi Complete!");
  Serial.println("System Finished Initializing.");
  Serial.println("PARCEL is waiting for a package.\n");
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
  delay(10);
  // Run executor periodically
  RCCHECK(rclc_executor_spin_some(&executor_sub, RCL_MS_TO_NS(100)));  // (10hz)
  RCCHECK(rclc_executor_spin_some(&executor_pub_odom, RCL_MS_TO_NS(100)));
  RCCHECK(rclc_executor_spin_some(&executor_pub_imu, RCL_MS_TO_NS(100)));

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

                // publish location to raspberry pi

                // publish to app 

                // update state
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
