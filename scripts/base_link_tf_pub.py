#!/usr/bin/env python

import rospy
import numpy as np
import tf2_ros as tf
import tf.transformations as tft
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header

# INITIALIZE NODE
rospy.init_node('base_link_tf_pub', anonymous=True)
r = rospy.Rate(10)  # 10hz

# GRAB USEFUL TF OBJECTS
br = tf.TransformBroadcaster()
buffer = tf.Buffer()
listener = tf.TransformListener(buffer)

while not rospy.is_shutdown():

    # 1) Get the current transform of the left_camera w.r.t. world. Use the TF tree!

    try:
        left_camera_transformation = buffer.lookup_transform("world", "left_cam", rospy.Time())
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue

    rotation = left_camera_transformation.transform.rotation
    translation = left_camera_transformation.transform.translation
    # print("translation: ", translation)
    # print("rotation: ", rotation)

    # 2) Convert the left camera's transform to a 4x4 numpy array.

    quaternion = tft.quaternion_matrix([
        rotation.x, rotation.y, rotation.z, rotation.w
    ])
    quaternion[0, 3] = translation.x
    quaternion[1, 3] = translation.y
    quaternion[2, 3] = translation.z

    # 3) Compute the current transform of the left camera w.r.t. world by composing the
    #    precomputed camera-base_link transform with the base_link-world transform.

    left_camera_pose_wrt_car = np.array([
        [1, 0, 0, -.05],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ])
    car_pose_wrt_left_camera = np.linalg.inv(left_camera_pose_wrt_car)
    car_pose_wrt_world = np.matmul(quaternion, car_pose_wrt_left_camera)

    base_link_gt_2 = TransformStamped(
        header=Header(stamp=rospy.Time.now(), frame_id="world"),
        child_frame_id="base_link_gt_2",
    )

    base_link_gt_2.transform.translation.x = car_pose_wrt_world[0, 3]
    base_link_gt_2.transform.translation.y = car_pose_wrt_world[1, 3]
    base_link_gt_2.transform.translation.z = car_pose_wrt_world[2, 3]
    base_link_gt_2.transform.rotation.x = tft.quaternion_from_matrix(car_pose_wrt_world)[0]
    base_link_gt_2.transform.rotation.y = tft.quaternion_from_matrix(car_pose_wrt_world)[1]
    base_link_gt_2.transform.rotation.z = tft.quaternion_from_matrix(car_pose_wrt_world)[2]
    base_link_gt_2.transform.rotation.w = tft.quaternion_from_matrix(car_pose_wrt_world)[3]

    # 5) Broadcast the computed transforms for the base_link_gt_2 to the TF tree

    br.sendTransform(base_link_gt_2)
