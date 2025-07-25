
#ifndef POINTCLOUD_TO_LASERSCAN__LASERSCAN_TO_POINTCLOUD_NODE_HPP_
#define POINTCLOUD_TO_LASERSCAN__LASERSCAN_TO_POINTCLOUD_NODE_HPP_

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "message_filters/subscriber.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/message_filter.h"
#include "tf2_ros/transform_listener.h"

#include "laser_geometry/laser_geometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include "visibility_control.h"

namespace pointcloud_to_laserscan
{
typedef tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan> MessageFilter;

//! \brief The PointCloudToLaserScanNodelet class to process incoming laserscans into pointclouds.
//!
class LaserScanToPointCloudNode : public rclcpp::Node
{
public:
  POINTCLOUD_TO_LASERSCAN_PUBLIC
  explicit LaserScanToPointCloudNode(const rclcpp::NodeOptions & options);

  ~LaserScanToPointCloudNode() override;

private:
  void scanCallback(sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);

  void subscriptionListenerThreadLoop();

  std::unique_ptr<tf2_ros::Buffer> tf2_;
  std::unique_ptr<tf2_ros::TransformListener> tf2_listener_;
  message_filters::Subscriber<sensor_msgs::msg::LaserScan> sub_;
  std::shared_ptr<rclcpp::Publisher<sensor_msgs::msg::PointCloud2>> pub_;
  std::unique_ptr<MessageFilter> message_filter_;
  std::thread subscription_listener_thread_;
  std::atomic_bool alive_{true};

  laser_geometry::LaserProjection projector_;

  // ROS Parameters
  int input_queue_size_;
  std::string target_frame_;
  double tolerance_;
};

}  // namespace pointcloud_to_laserscan

#endif  // POINTCLOUD_TO_LASERSCAN__LASERSCAN_TO_POINTCLOUD_NODE_HPP_