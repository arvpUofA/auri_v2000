# au_core

## Messages
Custom ros messages for the auv.

### Motors (msg/MC*.msg)
Message to transfer motor data. 

#### Raw Motor Speed (msg/MCRaw.msg)
Feedback from the motor node

##### Format
```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float32 horLeft
float32 horRight
float32 verLeft
float32 verRight
float32 strFront
float32 strBack
```

#### Setting Relative Motor Speed (msg/MCBaseSpeed.msg | msg/MCDiff.msg)

Set the base speed of the motor using `MCBaseSpeed`

##### Format
```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float32 baseSpeed
```

Set the differential between the two motors using `MCDiff`

##### Format
```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float32 differential
```

### Kill Switch (msg/KillSig.msg)

Status of the kill switch

##### Format
```
uint8 KILL_ENGAGE=0
uint8 KILL_DISENGAGE=1
uint8 kill
```

### Depth Sensor (msg/Depth.msg)

Data published from the MS5837 depth sensor. Absolute pressure is in Pascals and depth is in metres.

##### Format
```
std_msgs/Header header
  uint32 seq
  time stamp
  string frame_id
float32 abs_pressure
float32 depth
```

### Regions of Interest (msg/Roi*.msg)
Messages containing ROIs for the tracker

#### Single Region of Interest (msg/Roi.msg)

##### Format
```
# Identification information about the tracked ROI
string[] tags
float32 confidence
float32 actualHeight
float32 actualWidth

# Pixel locations in image as a complex polygon
geometry_msgs/Point[] polygon

# Bounding box for the polygon
geometry_msgs/Point   topLeft   # Aligned to the image
uint32 width                    # Bounding box width in pixels
uint32 height                   # Bounding box height in pixels

# Estimated position in real coordinates relative to the robot
# Displacement is represented in meters
# hasOrientation Indicates if there an orientation is specified
# in the pose
geometry_msgs/Pose poseEstimate
uint8 hasOrientation
```

#### Array of Regions of Interest (msg/RoiArray.msg)

##### Format
```
Roi[] regionsOfInterest
```

---

#### Camera rules file
###### 40-firefly.rules
```
SUBSYSTEM=="usb", ATTRS{idVendor}=="1e10", ATTRS{idProduct}=="2002", GROUP="plug dev", SYMLINK+="firefly", MODE:="0666"
```
## laptopGitStart.sh
Pulls or clones au_core, au_vision, and au_missioncontrol
 
##### Format
```
To use (while in scripts folder):
chmod u+x laptopGitStart.sh
./laptopGitStart.sh
```
## odroidGitStart.sh / tegraGitStart.sh
```
For pulling repos on the tegra/odroid
Usage is the same as laptopGitStart.sh

```

## all.sh
The fastest way fetch all of the ARVP code. Pulls all repos and installs all system dependencies. Usage: make sure you have [setup ssh keys on github](https://help.github.com/articles/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent/) then execute the following:
```
$ cd ~/catkin_ws/src/
$ git clone git@github.com:arvpUofA/au_core.git
$ ./au_core/scripts/all.sh
```


## ROS Bag
The `record_bag.launch` file is setup to save cameras and sensor
topics into separate bag files. Splits the camera bag files into
3 GB bags. The following args can be set
* *record_cameras*: true to enable bagging camera feeds
* *bag_name*: the name to assign the bag file. Bag files are automatically
prefixed with the current timestamp
* *path*: directory to store the bag files eg. /path/to/usb/drive

Example usage:

`roslaunch au_core record_bag.launch path:=/media/usb bag_name:=2017_07_30` 

The defaults are intended for logging on the TX2 using an external
SSD drive mounted at `/media/ssd`. To mount the ssd run:
```
sudo mount /dev/sda1 /media/ssd
```

The `extract_images_from_bag.launch` file is to extract images from a bag
file into a both a video, and a folder of images.
The following args can be set:
* *bag*:           bag file (absolute or relative to cwd)
* *fps*:           fps for the video file. Leave blank for similar rate to bag file
* *image*:         image topic
* *output_dir*:    directory images and video should be output. images are saved
                   to ${output_dir}/images/ . Video is saved to ${output_dir}/
* *image_name*:    Name prefix for the image. Defaults to the name of the bag file
                   Images are saved as ${image_name}%05d.jpg

Example usage:
Please take a look at the [launch file](https://github.com/arvpUofA/au_everything/tree/master/catkin_ws/src/au_core/launch/extract_images_from_bag.launch) for example usage/more info
