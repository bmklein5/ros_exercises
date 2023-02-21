#!/usr/bin/env python

import rospy
import tf2_ros as tf
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header

# INITIALIZE NODE AND BROADCASTER
rospy.init_node('static_tf_cam_publisher', anonymous=True)
br = tf.StaticTransformBroadcaster()

# CREATE TRANSFORMSTAMPED OBJECTS
left_camera = TransformStamped(
    header=Header(stamp=rospy.Time.now(), frame_id="base_link_gt"),
    child_frame_id="left_cam",
)
right_camera = TransformStamped(
    header=Header(stamp=rospy.Time.now(), frame_id="base_link_gt"),
    child_frame_id="right_cam",
)

# SET TRANSFORMS FOR LEFT
left_camera.transform.translation.x = -.05
left_camera.transform.translation.y = 0
left_camera.transform.translation.z = 0
left_camera.transform.rotation.x = 0
left_camera.transform.rotation.y = 0
left_camera.transform.rotation.z = 0
left_camera.transform.rotation.w = 1

# SET TRANSFORMS FOR RIGHT
right_camera.transform.translation.x = .05
right_camera.transform.translation.y = 0
right_camera.transform.translation.z = 0
right_camera.transform.rotation.x = 0
right_camera.transform.rotation.y = 0
right_camera.transform.rotation.z = 0
right_camera.transform.rotation.w = 1

# BROADCAST TRANSFORM
br.sendTransform([left_camera, right_camera])

rospy.spin()
