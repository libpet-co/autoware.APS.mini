#include "motor_controller.hpp"
#include "rclcpp/rclcpp.hpp"

MotorController::MotorController(rclcpp::Node* node) : 
    node_(node), 
    cmd_left_rpm_(0.0), 
    cmd_right_rpm_(0.0),
    left_speed_rpm_(0),
    right_speed_rpm_(0)
{
    // 声明参数
    node_->declare_parameter("serial_port", "/dev/ttyUSB0");
    node_->declare_parameter("baud_rate", 115200);
    node_->declare_parameter("wheel_diameter", 0.26);
    node_->declare_parameter("max_motor_speed", 50);
    node_->declare_parameter("accel_up_time", 0);
    node_->declare_parameter("accel_down_time", 0);
    
    // Sigmoid参数
    node_->declare_parameter("up_l", 0.0);
    node_->declare_parameter("up_h", 0.0);
    node_->declare_parameter("up_s", 0.0);
    node_->declare_parameter("up_c", 0.0);
    node_->declare_parameter("down_l", 0.0);
    node_->declare_parameter("down_h", 0.0);
    node_->declare_parameter("down_s", 0.0);
    node_->declare_parameter("down_c", 0.0);
    
    updateParameters();
    
}

MotorController::~MotorController()
{
    if (ros_ser_.isOpen()) {
        try {
            ros_ser_.close();
        } catch (const serial::IOException& e) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to close serial port: %s", e.what());
        }
    }
}

void MotorController::updateParameters()
{
    wheel_diameter_ = node_->get_parameter("wheel_diameter").as_double();
    max_motor_speed_ = node_->get_parameter("max_motor_speed").as_int();
    rpm_per_mps_ = 60.0 / (M_PI * wheel_diameter_);
    accel_up_time_ = node_->get_parameter("accel_up_time").as_int();
    accel_down_time_ = node_->get_parameter("accel_down_time").as_int();

    // 获取Sigmoid参数
    up_l_ = node_->get_parameter("up_l").as_double();
    up_h_ = node_->get_parameter("up_h").as_double();
    up_s_ = node_->get_parameter("up_s").as_double();
    up_c_ = node_->get_parameter("up_c").as_double();
    down_l_ = node_->get_parameter("down_l").as_double();
    down_h_ = node_->get_parameter("down_h").as_double();
    down_s_ = node_->get_parameter("down_s").as_double();
    down_c_ = node_->get_parameter("down_c").as_double();
}

void MotorController::initialize()
{
    initializeSerial();
    initializeMotors();
    enableMotors();
}

void MotorController::initializeSerial()
{
    try {
        const std::string port = node_->get_parameter("serial_port").as_string();
        const int baud = node_->get_parameter("baud_rate").as_int();
        ros_ser_.setPort(port);
        ros_ser_.setBaudrate(baud);
        serial::Timeout to = serial::Timeout::simpleTimeout(50);
        ros_ser_.setTimeout(to);
        ros_ser_.setBytesize(serial::eightbits);
        ros_ser_.setParity(serial::parity_none);
        ros_ser_.setStopbits(serial::stopbits_one);
        ros_ser_.open();
        ros_ser_.flush();

        if (ros_ser_.isOpen()) {
            RCLCPP_INFO(node_->get_logger(), "Serial port opened successfully at %d baud", baud);
        }
    } catch (serial::IOException &e) {
        RCLCPP_FATAL(node_->get_logger(), "Failed to open serial port: %s", e.what());
        throw;
    }
}

void MotorController::initializeMotors()
{
    // 设置为速度模式 (0x03)
    unsigned char command[8] = {0x01, 0x06, 0x20, 0x0D, 0x00, 0x03, 0x00, 0x00};
    calculateCrc(command, 8);
    sendCommand(command, 8);

    // 设置加速时间
    updateAccelTime();

    RCLCPP_INFO(node_->get_logger(), "Motors Initialized");
}

void MotorController::updateAccelTime()
{
    unsigned char accel_up_time_high = static_cast<unsigned char>((accel_up_time_ >> 8) & 0xFF);
    unsigned char accel_up_time_low = static_cast<unsigned char>(accel_up_time_ & 0xFF);
    unsigned char accel_down_time_high = static_cast<unsigned char>((accel_down_time_ >> 8) & 0xFF);
    unsigned char accel_down_time_low = static_cast<unsigned char>(accel_down_time_ & 0xFF);

    // 左电机S型加速时间
    unsigned char command1[8] = {0x01, 0x06, 0x20, 0x80, accel_up_time_high, accel_up_time_low, 0x00, 0x00};
    calculateCrc(command1, 8);
    sendCommand(command1, 8);

    // 右电机S型加速时间
    unsigned char command2[8] = {0x01, 0x06, 0x20, 0x81, accel_up_time_high, accel_up_time_low, 0x00, 0x00};
    calculateCrc(command2, 8);
    sendCommand(command2, 8);

    // 左电机S型减速时间
    unsigned char command3[8] = {0x01, 0x06, 0x20, 0x82, accel_down_time_high, accel_down_time_low, 0x00, 0x00};
    calculateCrc(command3, 8);
    sendCommand(command3, 8);

    // 右电机S型减速时间
    unsigned char command4[8] = {0x01, 0x06, 0x20, 0x83, accel_down_time_high, accel_down_time_low, 0x00, 0x00};
    calculateCrc(command4, 8);
    sendCommand(command4, 8);

    RCLCPP_INFO(node_->get_logger(), "Accelerate time updated");
}

void MotorController::enableMotors()
{
    unsigned char command[8] = {0x01, 0x06, 0x20, 0x0E, 0x00, 0x08, 0x00, 0x00};
    calculateCrc(command, 8);
    sendCommand(command, 8);
    RCLCPP_INFO(node_->get_logger(), "Motors Enabled");
}
void MotorController::disableMotors()
{
    try {
        unsigned char command[8] = {0x01, 0x06, 0x20, 0x0E, 0x00, 0x07, 0x00, 0x00};
        calculateCrc(command, 8);
        sendCommand(command, 8);
        RCLCPP_INFO(node_->get_logger(), "Motor Disabled");
    } 
    catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to disable motors: %s", e.what());
    }
}

void MotorController::setMotorSpeeds(int left_speed, int right_speed)
{

    // limit the range when speed up
    // if(-left_speed == right_speed){
    //     left_speed = std::clamp(left_speed, -max_motor_speed_, 40);
    //     right_speed = std::clamp(right_speed, -40, max_motor_speed_);
    // }
    

    // RCLCPP_INFO(node_->get_logger(), "---------------------------------------------------");
    // RCLCPP_INFO(node_->get_logger(), "target_left_motor_speed: %d | target_right_motor_speed: %d",
    //         left_speed, right_speed);
    // RCLCPP_INFO(node_->get_logger(), "cmd_left before increment: %f | cmd_right before increment: %f",
    //         cmd_left_rpm_, cmd_right_rpm_);

    if (left_speed * cmd_left_rpm_ < 0){
        cmd_left_rpm_ = calculateSigmoidIncrement(0.0,cmd_left_rpm_);
    }
    else{
        cmd_left_rpm_ = calculateSigmoidIncrement(left_speed,cmd_left_rpm_);
    }
    if (right_speed * cmd_right_rpm_ < 0){
        cmd_right_rpm_ = calculateSigmoidIncrement(0.0,cmd_right_rpm_);
    }
    else{
        cmd_right_rpm_ = calculateSigmoidIncrement(right_speed,cmd_right_rpm_);
    }
    // RCLCPP_INFO(node_->get_logger(), "cmd_left after increment: %f | cmd_right after increment: %f",
    //         cmd_left_rpm_, cmd_right_rpm_);

    int out_left = static_cast<int>(std::round(cmd_left_rpm_));
    int out_right = static_cast<int>(std::round(cmd_right_rpm_));


    // int out_left = static_cast<int>(std::round(left_speed));
    // int out_right = static_cast<int>(std::round(right_speed));

    // 发送命令
    sendMotorCommand(0x88, out_left);
    sendMotorCommand(0x89, out_right);

    // 清除串口缓冲区
    delayProcess(5);
    while (ros_ser_.available()) {
        unsigned char garbage_buffer[8];
        ros_ser_.read(garbage_buffer, 8);
    }
}

double MotorController::calculateSigmoidIncrement(double target_motor_rpm, double cmd_motor_rpm)
{
    bool is_acceleration = false;
    double effective_s = up_s_;
    double delta = target_motor_rpm - cmd_motor_rpm;
    double abs_delta = std::abs(delta);
    if (std::abs(delta) < 2 && target_motor_rpm == 0.0) {
        cmd_motor_rpm = 0.0;
        return cmd_motor_rpm;
    }
    
    else if(target_motor_rpm >= 0 && cmd_motor_rpm >= 0){
            is_acceleration = (delta > 0);
            if(!is_acceleration){effective_s = down_s_;}

    }
    else if(target_motor_rpm <= 0 && cmd_motor_rpm <= 0){
            is_acceleration = (delta < 0);
            if(!is_acceleration){effective_s = down_s_;}
    }

    double up_exponent = -effective_s * (abs_delta - up_c_  );
    double down_exponent = effective_s * (abs_delta - down_c_  );
    double up_sigmoid = 1.0 / (1.0 + std::exp(up_exponent));
    double down_sigmoid = 1.0 / (1.0 + std::exp(down_exponent));    

    double magnitude ;
    if(is_acceleration && target_motor_rpm >=0){
        magnitude = up_l_ + (up_h_ - up_l_) * up_sigmoid;
        if(cmd_motor_rpm + magnitude > target_motor_rpm){
            return target_motor_rpm;
        }
    }else if(is_acceleration && target_motor_rpm <= 0){
        magnitude = -up_l_ - (up_h_ - up_l_) * up_sigmoid;
        if(cmd_motor_rpm + magnitude < target_motor_rpm){
            return target_motor_rpm;
        }
    }
    else if(!is_acceleration && target_motor_rpm >=0 && cmd_motor_rpm >0){
        magnitude = -down_l_ - (down_h_ - down_l_) * down_sigmoid;
        if(cmd_motor_rpm + magnitude < target_motor_rpm){
            return target_motor_rpm;
        }
    }else if(!is_acceleration && target_motor_rpm <= 0 && cmd_motor_rpm <0){
        magnitude = down_l_ + (down_h_ - down_l_) * down_sigmoid;
        if(cmd_motor_rpm + magnitude > target_motor_rpm){
            return target_motor_rpm;
        }
    }

    // RCLCPP_INFO(node_->get_logger(), "delta: %f | absolute delta: %f", delta, abs_delta);
    // RCLCPP_INFO(node_->get_logger(), "effective_s: %f", effective_s);
    // RCLCPP_INFO(node_->get_logger(), "is_acceleration: %d", is_acceleration);
    // RCLCPP_INFO(node_->get_logger(), "magnitude: %f", magnitude);


    return cmd_motor_rpm += magnitude;
    // return cmd_motor_rpm; 
}

void MotorController::sendMotorCommand(unsigned char motor_address, int speed)
{
    unsigned char buffer[8] = {0x01, 0x06, 0x20, motor_address, 0x00, 0x64, 0x00, 0x00};


    // 速度限制
    if (speed > max_motor_speed_) speed = max_motor_speed_;
    else if (speed < -max_motor_speed_) speed = -max_motor_speed_;

    int16_t speed_value = static_cast<int16_t>(speed);
    buffer[4] = static_cast<unsigned char>((speed_value >> 8) & 0xFF);
    buffer[5] = static_cast<unsigned char>(speed_value & 0xFF);

    calculateCrc(buffer, 8);
    sendCommand(buffer, 8);
}


void MotorController::getCurrentSpeeds(double &linear_speed, double &angular_speed, double wheel_distance)
{
    while (ros_ser_.available()){
        unsigned char garbage_buffer[8]; // 临时缓冲区        
        ros_ser_.read(garbage_buffer, 8);
    }

    getMotorSpeeds();
    double left_mps = rpmToMps(left_speed_rpm_);
    double right_mps = rpmToMps(right_speed_rpm_);
    // kinematic forward
    linear_speed = (left_mps + right_mps) / 2.0;
    angular_speed = (right_mps - left_mps) / wheel_distance;
}

void MotorController::getMotorSpeeds()
{
    unsigned char request_buffer[8] = {0x01, 0x03, 0x20, 0xAB, 0x00, 0x02, 0xBE, 0x2B};
    calculateCrc(request_buffer, 8);
    sendCommand(request_buffer, 8);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    unsigned char response_buffer[9] = {0};
    size_t bytesRead = ros_ser_.read(response_buffer, 9);

    if (bytesRead == 9) {
        left_speed_rpm_ = -extractMotorSpeed(response_buffer[3], response_buffer[4]) /10;
        right_speed_rpm_ = extractMotorSpeed(response_buffer[5], response_buffer[6]) /10;

    } else {
        RCLCPP_WARN(node_->get_logger(), "Incomplete motor speed response: %zu bytes", bytesRead);
    }
}



double MotorController::rpmToMps(double rpm)
{
    return (rpm * M_PI * wheel_diameter_) / 60.0;
}

void MotorController::setSigmoidSpeeds(double target_left, double target_right)
{
    cmd_left_rpm_ = calculateSigmoidIncrement(target_left, cmd_left_rpm_);
    cmd_right_rpm_ = calculateSigmoidIncrement(target_right, cmd_right_rpm_);
}

void MotorController::sendCommand(const unsigned char command[], size_t length)
{
    try {
        if (!ros_ser_.isOpen()) {
            RCLCPP_ERROR(node_->get_logger(), "Serial port not open");
            return;
        }
        ros_ser_.write(command, length);
        delayProcess(5);
    } catch (const serial::IOException& e) {
        RCLCPP_ERROR(node_->get_logger(), "Serial write error: %s", e.what());
    }
}

void MotorController::calculateCrc(unsigned char buffer[], size_t length)
{
    if (length < 8) return;
    
    uint16_t crc = 0xFFFF;
    for (int i = 0; i < 6; i++) {
        crc ^= buffer[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x0001) {
                crc = (crc >> 1) ^ 0xA001;
            } else {
                crc >>= 1;
            }
        }
    }
    buffer[6] = crc & 0xFF;
    buffer[7] = (crc >> 8) & 0xFF;
}

void MotorController::delayProcess(int milliseconds)
{
    std::this_thread::sleep_for(std::chrono::milliseconds(milliseconds));
}

int16_t MotorController::extractMotorSpeed(uint8_t highByte, uint8_t lowByte)
{
    return (static_cast<int16_t>(highByte) << 8) | static_cast<uint8_t>(lowByte);
}