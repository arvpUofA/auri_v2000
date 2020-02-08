#!/usr/bin/env python
import rospy
import rosbag
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import cv2
import os


def main():
    rospy.init_node('extract_images_from_bag', disable_signals=True)

    bag_file = rospy.get_param('~bag')
    fps = rospy.get_param('~fps')
    image_topic = rospy.get_param('~image')
    output_dir = rospy.get_param('~output_dir')
    output_image_name = rospy.get_param('~image_name')
    image_dir_name = 'images'

    # ROS_HOME must be set for relative paths to work
    if os.getcwd().endswith('.ros'):
        rospy.logerr("Read usage instructions in the launch file")
        rospy.logerr(
            "Usage: `cwd` needs to use backticks (beside 1 on the keyboard)\nROS_HOME=`cwd` roslaunch au_core extract_images_bag.launch bag:=<bag_file>"
        )
        exit()

    if not bag_file:
        rospy.logerr("Please provide a bag file")
        exit()

    bag_name = os.path.splitext(os.path.basename(bag_file))[0]
    output_image_name = output_image_name if output_image_name else bag_name
    bag_path = os.path.abspath(bag_file)
    output_dir = os.path.abspath(output_dir)
    image_output_dir = os.path.join(output_dir, image_dir_name)

    # create output directory
    if not os.path.isdir(output_dir):
        os.mkdir(output_dir)

    # check if output directory already contains files
    if os.listdir(output_dir):
        rospy.logerr("%s is not an empty directory", output_dir)
        exit()

    if not os.path.isdir(image_output_dir):
        os.mkdir(image_output_dir)

    bridge = CvBridge()
    bag = rosbag.Bag(bag_path)

    # get initial information for the OpenCV's VideoWriter
    image_count = bag.get_type_and_topic_info()[1][image_topic][1]
    fps = fps if fps else bag.get_type_and_topic_info()[1][image_topic][3]
    topic, msg, t = next(bag.read_messages(topics=[image_topic]))
    rospy.loginfo("%d %s messages were found", image_count, image_topic)

    video_writer = cv2.VideoWriter(
        os.path.join(output_dir, '{}.mp4'.format(bag_name)),
        cv2.VideoWriter_fourcc(*'mp4v'), fps, (msg.width, msg.height))

    # write image to images folder and video
    index = 0
    for topic, msg, t in bag.read_messages(topics=[image_topic]):
        try:
            image_name = '{}{:05d}.jpg'.format(output_image_name, index)
            cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
            cv2.imwrite(os.path.join(image_output_dir, image_name), cv_image)
            video_writer.write(cv_image)
            index += 1
        except CvBridgeError as e:
            print(e)

    video_writer.release()
    bag.close()


if __name__ == '__main__':
    main()
