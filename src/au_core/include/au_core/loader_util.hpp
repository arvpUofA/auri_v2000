#pragma once

// #include <ros/package.h>
#include <yaml-cpp/yaml.h>
#include <optional>
#include <string>

namespace au_core {

struct ObjectSize {
  double width;   // m
  double length;  // m
  double height;  // m
};

//! reads au_core/params/topics.yaml
//! \param topic_key
//! \return topic name
//! \throw std::runtime_error if key does not exist
std::string load_topic(const std::string& topic_key);

//! reads au_core/params/frames.yaml
//! \param frame_key hierarchy can be defined using /
//! \return frame name
//! \throw std::runtime_error if key does not exist
std::string load_frame(const std::string& frame_key);

//! reads au_core/params/scene_objects.yaml
//! \param obj_name name of the scene object
//! \param local If true, pulls object dimensions built by the team. If false,
//! returns competition dimensions
std::optional<ObjectSize> load_object_size(const std::string& obj_name,
                                           bool local);

}  // namespace au_core
