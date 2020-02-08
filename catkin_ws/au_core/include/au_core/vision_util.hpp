#ifndef __AU_VISION_UTIL__
#define __AU_VISION_UTIL__

#include <au_core/camera_info.h>
#include <string>

namespace au_core {

/**
 * @breif Calculates the distance between the camera and a target object
 * @param width The width of the target on the image (in pixels)
 * @param height The height of the target on the image (in pixels)
 * @param actual_width The actual width of the target object (in units)
 * @param actual_height The actual height of the target object (in units)
 * @param camera_info Object containing the camera parameters
 *
 * @return The distance (in units) away from the camera to the target object
 *
 * Uses the camera focal lengths and the actual width and height to
 * estimate the distance to the target object
 *
 * Notes:
 *  - If any of the camera focal lengths or widths/heights provided are 0,
 *    the function returns a default distance of 0
 *  - If input axis is invalid, the function returns 0
 *  - By convention the units are in cm
 */

double calculateDistance(uint32_t perceived_width, double actual_width,
                         double focal_length);

/**
 * @breif Calculates the distance between the camera and a target object
 * @param perceived_width The width of the target on the image (in pixels)
 * @param perceived_height The height of the target on the image (in pixels)
 * @param actual_width The actual width of the target object (in units)
 * @param actual_height The actual height of the target object (in units)
 * @param camera_info Object containing the camera parameters
 *
 * @return The distance (in units) away from the camera to the target object
 *
 * Shortcut for averaging distance calculations in the X and Y axis
 */

double calculateDistance(uint32_t perceived_width, uint32_t perceived_height,
                         double actual_width, double actual_height,
                         const CameraInfo& camera_info);

/**
 * @breif Calculates the distance in the image plane to a target
 * @param pixel_error The pixel distance between the image centerpoint and the
 * target
 * @param focal_length The focal length for the lateral axis
 * @param distance The distance to the target
 *
 * @return The lateral distance (in units) to a target in the image frame
 *
 * Calculates the distance to a target on the lateral (X or Y) axis in an image
 * Formula:
 *      W = D' x P / F
 * Legend:
 *     P: Perceived width
 *     W: Actual Width
 *     F: Focal length
 *     D': Actual Distance
 */

double calculateLateral(int pixel_error, double focal_length, double distance);

/**
 * @breif Calculates the angular distance to a target on the image plane
 * @param pixel_error The pixel distance between the image centerpoint and the
 * target
 * @param image_width The width of the image for the lateral axis (X or Y)
 * @param fov The field of view (in degrees / radians) of the camera
 *
 * @return The angle to the target in degrees / radians
 *
 * Calculates the angle to a target in an image by normalizing the pixel_error
 * in range (-image_width/2, image_width/2) to the camera's angular range
 * (-fov/2, fov/2).
 */

double calculateAngle(int pixel_error, int image_width, int fov);

}  // namespace au_core

#endif
