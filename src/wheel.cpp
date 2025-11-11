#include <Arduino.h>

#define MAX_WHEEL_ANGULAR_SPEED 15.71  // rad/s (2 * pi * 150RPM) / 60s
#define MAX_PWM 255
#define WHEEL_RADIUS 0.045   // meters

class Wheel {
public:
    Wheel(float kp, float ki, float kd, float radius)
        : Kp(kp), Ki(ki), Kd(kd), wheel_radius(radius),
          integral(0), prev_error(0) {}

    int curr_pwm;

    // Call every control loop]
    int computePWM(float target_speed, float measured_speed, float dt) {
        // --- PID math ---
    float error = target_speed - measured_speed;
    integral += (dt * (error + prev_error) / 2.0);   // trapezoidal integration
    float derivative = (error - prev_error) / dt;
    prev_error = error;

    double control_effort = (Kp * error) + (Ki * integral) + (Kd * derivative);

    // --- Convert linear velocity PID output to wheel angular velocity ---
    float omega_cmd = control_effort / WHEEL_RADIUS;  // rad/s

    // --- Scale to PWM ---
    float scaled = omega_cmd / MAX_WHEEL_ANGULAR_SPEED;
    if (scaled > 1.0) scaled = 1.0;
    else if (scaled < -1.0) scaled = -1.0;

    int pwm = (int)(scaled * MAX_PWM);

    return pwm;
    }

    int getDirection(float target_speed) {
        return (target_speed >= 0) ? HIGH : LOW;
    }

private:
    float Kp, Ki, Kd;
    float wheel_radius;
    float integral;
    float prev_error;
};
