#include <chrono>
#include <functional>
#include <memory>
#include <string>

// ==============linux driver===============
#include     <stdio.h>      /*标准输入输出的定义*/
#include     <stdlib.h>     /*标准函数库定义*/
#include     <unistd.h>     /*UNIX 标准函数定义*/
#include     <sys/types.h>  /**/
#include     <sys/stat.h>  
#include     <fcntl.h>	    /*文件控制定义*/
#include     <termios.h>    /*PPSIX 终端控制定义*/
#include     <errno.h>      /*错误号定义*/
#include     <sys/time.h>
#include     <getopt.h>

// ==============ros driver================
#include <serial/serial.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "nav_msgs/msg/path.hpp"
#include "geometry_msgs/msg/accel.hpp"

#include "yesense_interface/msg/imu_data.hpp"


#include "yesense_decoder_comm.h" 
#include "yesense_decoder.h"

// ===================================================================
#define CNT_PER_SECOND					1000u

#define UART_RX_BUF_LEN					512u
#define DEG_TO_RAD_FACTOR				57.29577952383886

#define BAUDRATE_CNT					8

// ===================================================================
// ====== enumeration ======
typedef enum
{
	serial_drv_ros 		= 1,
	serial_drv_linux	= 2,
	serial_drv_unknown 	= -10
}serial_drv_t;

// ====== structure ======
typedef struct
{
	unsigned int flg;
	unsigned int timing_cnt;
	unsigned int msg_cnt;	
	unsigned int msg_rate;
}user_info_t;


// ===================================================================
using namespace std::chrono_literals;
using namespace yesense;

int g_baudrate_table[BAUDRATE_CNT] =
{
	9600, 19200, 38400, 57600, 115200,
	230400, 460800, 921600
};	// 目前仅支持这些波特率 

int g_baudrate_table_linux[BAUDRATE_CNT] =
{
	B9600, B19200, B38400, B57600, B115200,
	B230400, B460800, B921600
};	// 目前仅支持这些波特率 

// ===================================================================
speed_t obt_baudrate_from_int_to_linux(int baudrate)
{
	speed_t speed = B0;
	unsigned short i = 0u;

	for(i = 0u; i < BAUDRATE_CNT; i++)
	{
		if(baudrate == g_baudrate_table[i])
		{
			speed = g_baudrate_table_linux[i];
		}
	}

	return speed;
}

class YESENSE_Publisher : public rclcpp::Node
{
	public:
	YESENSE_Publisher()
	: Node("yesense_publisher")
	{
		driver_type = serial_drv_unknown;

		this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
		this->declare_parameter<int>("baud_rate", 460800);
		this->declare_parameter<std::string>("frame_id", "basic_id");
		this->declare_parameter<std::string>("driver_type", "ros_serial");
		this->declare_parameter<std::string>("imu_topic_ros", "imu_data_ros");
		this->declare_parameter<std::string>("imu_topic", "imu_data");
		
		std::string driver_type_str,imu_topic_ros,imu_topic;
		this->get_parameter("serial_port", serial_port);
		this->get_parameter("baud_rate", baud_rate);
		this->get_parameter("frame_id", frame_id);
		this->get_parameter("driver_type", driver_type_str);	
		this->get_parameter("imu_topic_ros", imu_topic_ros);
		this->get_parameter("imu_topic", imu_topic);	
		RCLCPP_INFO(this->get_logger(), "serial port %s\n", serial_port.c_str());
		RCLCPP_INFO(this->get_logger(), "baudrate %d\n", baud_rate);
		RCLCPP_INFO(this->get_logger(), "frame id %s\n", frame_id.c_str());
		RCLCPP_INFO(this->get_logger(), "driver type %s\n", driver_type_str.c_str());
		pub_imu_ros 		= this->create_publisher<sensor_msgs::msg::Imu>(imu_topic_ros.c_str(), 10);				
		pub_pose_geometry 	= this->create_publisher<geometry_msgs::msg::PoseStamped>("pose_geo", 10);		
		
		// =================================================
		*((unsigned int *)&yis_out) = 0u;		
		user_info.flg 		= 1u;
		user_info.timing_cnt 	= 0u;
		user_info.msg_cnt		= 0u;
		user_info.msg_rate	= 0u;

		try
		{				
			ser.setPort(serial_port);
			ser.setBaudrate(baud_rate);
			
			//串口设置
			serial::Timeout to = serial::Timeout::simpleTimeout(1000);
			ser.setTimeout(to);

			ser.setStopbits(serial::stopbits_t::stopbits_one);
			ser.setBytesize(serial::bytesize_t::eightbits);
			ser.setParity(serial::parity_t::parity_none);       //设置校验位

			//打开
			ser.open();		
			ser.flushInput();
		}
		catch (serial::IOException &e)
		{
			RCLCPP_INFO(this->get_logger(), "Unable to open port ");
			return;
		}	

		// =================================================
		timer_ 			= this->create_wall_timer(5ms, std::bind(&YESENSE_Publisher::timer_callback, this));
		timer_msg_rate_ = this->create_wall_timer(100ms, std::bind(&YESENSE_Publisher::callback_msg_rate_calc, this));		
	}

	~YESENSE_Publisher()
	{			
		ser.close();
	}

	private:
	void timer_callback()
	{
		std::vector<unsigned char> r_buffer_vec;  // 使用动态向量缓冲区
		size_t bytes_available = 0;
		size_t bytes_read = 0;

		try {
			bytes_available = ser.available();
			if (bytes_available > 0) {
				r_buffer_vec.resize(std::min(bytes_available, static_cast<size_t>(UART_RX_BUF_LEN)));  // 限制最大读取512
				bytes_read = ser.read(r_buffer_vec.data(), r_buffer_vec.size());
			}

			if (bytes_read == 0) {
				return;
			}

			int ret = decoder.data_proc(r_buffer_vec.data(), static_cast<unsigned int>(bytes_read), &yis_out);
			if(analysis_ok == ret)
			{
				if(yis_out.content.valid_flg)
				{
					yis_out.content.valid_flg = 0u;
					user_info.msg_cnt++;						
					publish_msg(&yis_out);
				}
			} else {
				// RCLCPP_WARN(this->get_logger(), "Data decode failed: ret=%d", ret);
			}
		} catch (const std::exception& e) {
			RCLCPP_FATAL(this->get_logger(), "Exception in timer_callback: %s", e.what());
			rclcpp::shutdown();
		}
	}
	
	void callback_msg_rate_calc()
	{
		if(user_info.flg)
		{
			user_info.timing_cnt++;
			if(user_info.timing_cnt >= CNT_PER_SECOND)
			{
				user_info.timing_cnt 	= 0u;
				user_info.msg_rate	= user_info.msg_cnt;
				user_info.msg_cnt		= 0u;
			}
		}
	}

	void publish_msg(yis_out_data_t *result);

	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::TimerBase::SharedPtr timer_msg_rate_;	

	// serial
	unsigned char r_buffer[UART_RX_BUF_LEN];
	serial::Serial ser;	// ros驱动串口	

	// configuration
	std::string serial_port;
	int baud_rate;
	std::string frame_id;	
	int driver_type;	// 选择使用ROS串口驱动或是linux原生驱动
	double timestamp_gap{0};	// 记录H30首帧数据里，模块精振时钟与操作系统时钟的gap

	// ===
	yis_out_data_t yis_out;
	user_info_t user_info;

	// ros2 original topic
	rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_ros;		
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_geometry;	

	yesense_decoder decoder;
};

void YESENSE_Publisher::publish_msg(yis_out_data_t *result)
{
	sensor_msgs::msg::Imu					imu_ros_data;
   	geometry_msgs::msg::PoseStamped 		pose_data;	

    // =========== publish imu message ==============
	// 用sensor精振和系统时间的gap修正IMU数据时间
	double now_timestamp = this->get_clock()->now().seconds();
	double sensor_timestamp = static_cast<double>(result->sample_timestamp / 1000000.0);
	if (std::abs(now_timestamp - (sensor_timestamp + timestamp_gap)) > 0.005) {
		timestamp_gap = now_timestamp - sensor_timestamp;
	}
	double amend_timestamp = sensor_timestamp + timestamp_gap;

	// imu ros data
	constexpr double ACCEL_COVARIANCE = 1.454e-5;   // (m/s²)²
	constexpr double GYRO_COVARIANCE = 3.731e-7;    // (rad/s)²
	
    imu_ros_data.header.frame_id 	= frame_id;		// 如果不加frame_id，rziv会因为frame_id为空而过滤不显示
    imu_ros_data.header.stamp = rclcpp::Time(static_cast<int64_t>(amend_timestamp * 1000000000.0)); // this->get_clock()->now();

	// 设置协方差矩阵（行优先顺序）
	// 加速度计协方差（对角矩阵）
	imu_ros_data.linear_acceleration_covariance = {
		ACCEL_COVARIANCE, 0.0, 0.0,
		0.0, ACCEL_COVARIANCE, 0.0,
		0.0, 0.0, ACCEL_COVARIANCE
	};

	// 陀螺仪协方差（对角矩阵）
	imu_ros_data.angular_velocity_covariance = {
		GYRO_COVARIANCE, 0.0, 0.0,
		0.0, GYRO_COVARIANCE, 0.0,
		0.0, 0.0, GYRO_COVARIANCE
	};

	// 姿态协方差（未知设置为-1）
	imu_ros_data.orientation_covariance = {
		-1.0, 0.0, 0.0,
		0.0, 0.0, 0.0,
		0.0, 0.0, 0.0
	};
	
    imu_ros_data.orientation.x = result->quat.q1;
	imu_ros_data.orientation.y = result->quat.q2;
   	imu_ros_data.orientation.z = result->quat.q3;
   	imu_ros_data.orientation.w = result->quat.q0;      
 
    imu_ros_data.angular_velocity.x = result->gyro.x / DEG_TO_RAD_FACTOR;
    imu_ros_data.angular_velocity.y = result->gyro.y / DEG_TO_RAD_FACTOR;
    imu_ros_data.angular_velocity.z = result->gyro.z / DEG_TO_RAD_FACTOR;
    imu_ros_data.linear_acceleration.x = result->acc.x;
    imu_ros_data.linear_acceleration.y = result->acc.y;
    imu_ros_data.linear_acceleration.z = result->acc.z;
    pub_imu_ros->publish(imu_ros_data);

  	// ================ update pose =================
    pose_data.header.frame_id 	= frame_id;
    pose_data.header.stamp		= imu_ros_data.header.stamp;
    pose_data.pose.position.x 	= 0.0;
    pose_data.pose.position.y 	= 0.0;
    pose_data.pose.position.z 	= 0.0;
    pose_data.pose.orientation.w = imu_ros_data.orientation.w;
    pose_data.pose.orientation.x = imu_ros_data.orientation.x;
    pose_data.pose.orientation.y = imu_ros_data.orientation.y;
    pose_data.pose.orientation.z = imu_ros_data.orientation.z;
   	pub_pose_geometry->publish(pose_data);
}

int main(int argc, char * argv[])
{
	rclcpp::init(argc, argv);  
  
	rclcpp::spin(std::make_shared<YESENSE_Publisher>());
	rclcpp::shutdown();
	
		
	return 0;
}