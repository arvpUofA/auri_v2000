#include <au_core/camera_info.h>

namespace au_core {

CameraInfo::CameraInfo()
    : fov(120), width(0), height(0), fx(0), fy(0), cx(0), cy(0) {}

// The K matrix should be sufficient for monocular cameras
//     [fx  0 cx]
// K = [ 0 fy cy]
//     [ 0  0  1]

CameraInfo::CameraInfo(const sensor_msgs::CameraInfo& camera_info)
    : fov(120),
      width(camera_info.width),
      height(camera_info.height),
      fx(camera_info.K[0]),
      fy(camera_info.K[4]),
      cx(camera_info.K[2]),
      cy(camera_info.K[5]) {}

CameraInfo fetchCameraInfo(ros::NodeHandle& nh, std::string info_topic,
                           double timeout) {
  auto info_msg = ros::topic::waitForMessage<sensor_msgs::CameraInfo>(
      info_topic, nh, ros::Duration(timeout));

  if (info_msg == nullptr) {
    ROS_WARN_STREAM("Could not retreive camera info from " << info_topic);
    return CameraInfo();
  }

  ROS_INFO_STREAM("Received camera info from " << info_topic);

  return CameraInfo(*info_msg);
}

}  // namespace au_core
