#include <au_core/vision_util.hpp>
#include <cmath>
#include <cstdint>

namespace au_core {

// D' = W x F / P -- P: Perceived width -- W: Actual Width -- F: Focal length --
// D': Actual Distance
double calculateDistance(uint32_t perceived_width, double actual_width,
                         double focal_length) {
  if (perceived_width == 0) return 0;

  double distance = actual_width * focal_length / (double)perceived_width;

  return distance;
}

double calculateDistance(uint32_t perceived_width, uint32_t perceived_height,
                         double actual_width, double actual_height,
                         const CameraInfo& camera_info) {
  if (actual_width == 0 || actual_height == 0 || camera_info.fx == 0 ||
      camera_info.fy == 0)
    return 0.0f;

  double xdistance =
      calculateDistance(perceived_width, actual_width, camera_info.fx);
  double ydistance =
      calculateDistance(perceived_height, actual_height, camera_info.fy);

  // Take an average for increased accuracy
  return (xdistance + ydistance) / 2.0;
}

double calculateLateral(int pixel_error, double focal_length, double distance) {
  if (focal_length == 0) return 0.0f;

  // W = D' x P / F -- P: Perceived width -- W: Actual Width -- F: Focal length
  // -- D': Actual Distance
  double lateral = distance * pixel_error / focal_length;

  return lateral;
}

double calculateAngle(int pixel_error, int image_width, int fov) {
  if (image_width == 0) return 0.0f;

  return (pixel_error * fov) / static_cast<double>(image_width * 2);
}

}  // namespace au_core
