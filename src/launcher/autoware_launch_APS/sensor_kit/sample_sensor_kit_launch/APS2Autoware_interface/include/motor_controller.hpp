#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include <serial/serial.h>
#include <rclcpp/rclcpp.hpp>
#include <algorithm>
#include <chrono>
#include <thread>
#include <iomanip>
#include <sstream>

class MotorController
{
public:
    MotorController(rclcpp::Node* node);
    ~MotorController();

    void initialize();
    void handle_button_press(int button_id, bool pressed);
    void handle_axis_values(const std::vector<float>& axes);
    void enableMotors();
    void disableMotors();
    void setMotorSpeeds(int left_speed, int right_speed);
    void getMotorSpeeds();
    void updateParameters();
    double rpmToMps(double rpm);
    void setSigmoidSpeeds(double target_left, double target_right);
    void getCurrentSpeeds(double &linear_speed, double &angular_speed, double wheel_distance);

    double rpm_per_mps_;  

private:
    rclcpp::Node* node_;
    serial::Serial ros_ser_;

    double wheel_diameter_;
    int max_motor_speed_;
    int accel_up_time_;
    int accel_down_time_;

    // Sigmoid参数
    double up_l_, up_h_, up_s_, up_c_;
    double down_l_, down_h_, down_s_, down_c_;

    // 当前指令RPM（用于Sigmoid平滑）
    double cmd_left_rpm_;
    double cmd_right_rpm_;

    // 当前实际RPM
    int16_t left_speed_rpm_;
    int16_t right_speed_rpm_;

    void initializeSerial();
    void initializeMotors();
    void sendMotorCommand(unsigned char motor_address, int speed);
    void sendCommand(const unsigned char command[], size_t length);
    void calculateCrc(unsigned char buffer[], size_t length);
    void delayProcess(int milliseconds);
    double calculateSigmoidIncrement(double target_motor_rpm, double cmd_motor_rpm);
    void updateAccelTime();
    int16_t extractMotorSpeed(uint8_t highByte, uint8_t lowByte);
};

#endif // MOTOR_CONTROLLER_H