#include <au_core/loader_util.hpp>

#include <ros/common.h>
#include <iostream>
#include <regex>

YAML::Node load_file_safe(const std::string& path) {
  try {
    return YAML::LoadFile(path);
  } catch (std::exception& e) {
    throw std::runtime_error("Unable to open yaml file: " + path);
  }
}

std::string au_core::load_topic(const std::string& topic_key) {
  static const std::string topics_path =
      ros::package::getPath("au_core") + "/params/topics.yaml";
  static const YAML::Node topics_root = load_file_safe(topics_path);
  try {
    return topics_root[topic_key].as<std::string>();
  } catch (YAML::RepresentationException& e) {
    throw std::runtime_error("invalid topic key: " + topic_key);
  }
}

std::string au_core::load_frame(const std::string& frame_key) {
  static const std::string frames_path =
      ros::package::getPath("au_core") + "/params/frames.yaml";
  static const YAML::Node frame_root = load_file_safe(frames_path);

  static const std::regex re{"/"};
  std::vector<std::string> frame_path{
      std::sregex_token_iterator(frame_key.begin(), frame_key.end(), re, -1),
      std::sregex_token_iterator()};

  YAML::Node frame = Clone(frame_root);
  try {
    for (const auto& f : frame_path) {
      frame = frame[f];
    }
    return frame.as<std::string>();
  } catch (YAML::RepresentationException& e) {
    throw std::runtime_error("invalid frame key: " + frame_key);
  }
}

template <>
struct YAML::convert<au_core::ObjectSize> {
  static bool decode(const Node& node, au_core::ObjectSize& size) {
    if (!node.IsMap()) return false;

    size.width = node["width"].as<double>();
    size.height = node["height"].as<double>();
    size.length = node["length"].as<double>();

    if (size.width <= 0 || size.height <= 0 || size.length <= 0) {
      ROS_ERROR("Invalid object dimension detected");
      return false;
    }

    return true;
  }
};

std::optional<au_core::ObjectSize> au_core::load_object_size(
    const std::string& obj_name, bool local) {
  static const std::string scene_objects_path =
      ros::package::getPath("au_core") + "/params/scene_objects.yaml";
  static const YAML::Node objects_root = load_file_safe(scene_objects_path);

  YAML::Node obj_node = objects_root[obj_name];
  if (!obj_node) {
    return {};
  }

  YAML::Node size_node = local ? obj_node["local"] : obj_node["comp"];

  return size_node.as<ObjectSize>();
}
