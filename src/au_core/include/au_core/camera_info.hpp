/**
 * @file camera_info.h
 * @author James Hryniw
 * @date 28 Aug 2017
 * @brief Header for the CameraInfo class
 *
 * Helper struct for camera information. Helps query sensor_msgs::CameraInfo
 * messages
 *
 * Subscribes to:
 *  * any topic of type [sensor_msgs::CameraInfo]
 *
 * Publishes to: N/A
 *
 */

#ifndef __CAMERA_INFO_H__
#define __CAMERA_INFO_H__

#include <sstream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>

namespace au_core {

/**
 *  @class CameraInfo
 *  @ingroup au_vision
 *  @brief Class PathDetector detector to detect paths
 */
struct CameraInfo {
  /** Contructor, initializes with 0s for all but @fov, which defaults to 120 */
  CameraInfo();

  explicit CameraInfo(const sensor_msgs::msg::CameraInfo& camera_info);

  ~CameraInfo() = default;

  /** Camera's field of view */
  int fov;

  /** Image width in pixels */
  int width;
  /** Image height in pixels */
  int height;

  /** Focal length in the x (horizontal) direction */
  double fx;
  /** Focal length in the y (vertical) direction */
  double fy;

  /** x coordinate of the camera's principal point */
  double cx;
  /** y coordinate of the camera's principal point */
  double cy;
};

/*
 * @breif Write to object from camera info topic at @info_topic
 * @param nh Nodehandle instance
 * @param info_topic Topic publishing the camera info
 */
CameraInfo fetchCameraInfo(ros::NodeHandle& nh, std::string info_topic,
                           double timeout = 5.0);

}  // namespace au_core

#endif
