######################################################
## Specifies a Region of Interest (ROI) in an image ##
######################################################

# camera enums 
uint8 FRONT=0
uint8 BOTTOM=1

# Information about the image origin
uint8 camera

# Identification information about the ROI
string[] tags
float32 confidence       # confidence that the detection is correct (0..1)
float32 height_confidence # confidence in the ROI width (0..1)
float32 width_confidence  # confidence in the ROI height (0..1)

# Pixel locations in image as complex polygons
geometry_msgs/Polygon[] polygons
uint8 coloring # coloring for polygons on debug view

# Bounding box for the polygon
geometry_msgs/Point   top_left   # Aligned to the image
uint32 width                    # Bounding box width in pixels
uint32 height                   # Bounding box height in pixels

# Estimated position in real coordinates relative to the robot
# Displacement is represented in meters
# hasOrientation Indicates if there an orientation is specified
# in the pose
geometry_msgs/Pose pose_estimate
uint8 has_orientation
