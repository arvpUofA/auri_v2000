#include <au_core/camera_info.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include "std_msgs/msg/string.hpp"
#include <rclcpp/rclcpp.hpp>

namespace au_core {

sensor_msgs::msg::CameraInfo::SharedPtr cam_msg=nullptr;

void cb(const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
  cam_msg=msg;
}

void waitForMessage(
  std::string topic, rclcpp::Node::SharedPtr node,
  std::chrono::duration<long int> time
) {
  const rclcpp::QoS & qos = rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_system_default));
  rclcpp::executors::SingleThreadedExecutor::SharedPtr exec=rclcpp::executors::SingleThreadedExecutor::make_shared();
  auto subber=node->create_subscription<sensor_msgs::msg::CameraInfo>(topic, qos, cb);
  exec->add_node(node);

  // Only works when you call it twice for some reason
  exec->spin_once(time);
  exec->spin_once(time);
  exec->remove_node(node);
}

CameraInfo::CameraInfo()
    : fov(120), width(0), height(0), fx(0), fy(0), cx(0), cy(0) {}

// The K matrix should be sufficient for monocular cameras
//     [fx  0 cx]
// K = [ 0 fy cy]
//     [ 0  0  1]

CameraInfo::CameraInfo(const sensor_msgs::msg::CameraInfo& camera_info)
    : fov(120),
      width(camera_info.width),
      height(camera_info.height),
      fx(camera_info.k[0]),
      fy(camera_info.k[4]),
      cx(camera_info.k[2]),
      cy(camera_info.k[5]) {}

CameraInfo fetchCameraInfo(rclcpp::Node::SharedPtr nh, std::string info_topic,
                           std::chrono::duration<long int> timeout) {
  waitForMessage(info_topic, nh, timeout);
  auto helper_node=rclcpp::Node::make_shared("camera_info");

  if (cam_msg == nullptr) {
    std::string warn_message="Could not retreive camera info from ";
    warn_message=warn_message.append(info_topic);

    RCLCPP_WARN(helper_node->get_logger(),warn_message);
    return CameraInfo();
  }

  // ROS_INFO_STREAM("Received camera info from " << info_topic);

  return CameraInfo(*cam_msg);
}

}  // namespace au_core
