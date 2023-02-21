#!/usr/bin/env python

import rospy
import numpy as np
import tf2_ros as tf
import tf.transformations as tft
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Header

# INITIALIZE NODE
rospy.init_node('dynamic_tf_cam_publisher', anonymous=True)
r = rospy.Rate(10)  # 10hz

# GRAB USEFUL TF OBJECTS
br = tf.TransformBroadcaster()
buffer = tf.Buffer()
listener = tf.TransformListener(buffer)

while not rospy.is_shutdown():

    # 1) Get the current transform of the robot w.r.t. world. Use the TF tree!

    try:
        transformation = buffer.lookup_transform("world", "base_link_gt", rospy.Time())
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        continue

    rotation = transformation.transform.rotation
    translation = transformation.transform.translation
    # print("translation: ", translation)
    # print("rotation: ", rotation)

    # 2) Convert the robot's transform to a 4x4 numpy array.

    quaternion = tft.quaternion_matrix([
        rotation.x, rotation.y, rotation.z, rotation.w
    ])
    quaternion[0, 3] = translation.x
    quaternion[1, 3] = translation.y
    quaternion[2, 3] = translation.z
    # print("quaternion: ", quaternion)

    # 3) Compute the current transform of the left camera w.r.t. world by composing the
    #    precomputed camera-base_link transform with the base_link-world transform.

    left_camera_pose_wrt_car = np.array([
        [1, 0, 0, -.05],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ])
    left_camera_pose_wrt_world = np.matmul(quaternion, left_camera_pose_wrt_car)
    left_camera = TransformStamped(
        header=Header(stamp=rospy.Time.now(), frame_id="world"),
        child_frame_id="left_cam",
    )

    left_camera.transform.translation.x = left_camera_pose_wrt_world[0, 3]
    left_camera.transform.translation.y = left_camera_pose_wrt_world[1, 3]
    left_camera.transform.translation.z = left_camera_pose_wrt_world[2, 3]
    # left_camera.transform.rotation = tft.quaternion_from_matrix(left_camera_pose_wrt_world)
    left_camera.transform.rotation.x = tft.quaternion_from_matrix(left_camera_pose_wrt_world)[0]
    left_camera.transform.rotation.y = tft.quaternion_from_matrix(left_camera_pose_wrt_world)[1]
    left_camera.transform.rotation.z = tft.quaternion_from_matrix(left_camera_pose_wrt_world)[2]
    left_camera.transform.rotation.w = tft.quaternion_from_matrix(left_camera_pose_wrt_world)[3]

    # 4) Compute the current transform of the right camera w.r.t the left camera by
    #    composing the relevant matrices.

    right_camera_pose_wrt_left_camera = np.array([
        [1, 0, 0, .1], # .1 cuz its wrt to the left camera
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ])
    right_camera = TransformStamped(
        header=Header(stamp=rospy.Time.now(), frame_id="left_cam"),
        child_frame_id="right_cam",
    )

    right_camera.transform.translation.x = right_camera_pose_wrt_left_camera[0, 3]
    right_camera.transform.translation.y = right_camera_pose_wrt_left_camera[1, 3]
    right_camera.transform.translation.z = right_camera_pose_wrt_left_camera[2, 3]
    # right_camera.transform.rotation = tft.quaternion_from_matrix(right_camera_pose_wrt_left_camera)
    right_camera.transform.rotation.x = tft.quaternion_from_matrix(right_camera_pose_wrt_left_camera)[0]
    right_camera.transform.rotation.y = tft.quaternion_from_matrix(right_camera_pose_wrt_left_camera)[1]
    right_camera.transform.rotation.z = tft.quaternion_from_matrix(right_camera_pose_wrt_left_camera)[2]
    right_camera.transform.rotation.w = tft.quaternion_from_matrix(right_camera_pose_wrt_left_camera)[3]

    # 5) Broadcast the computed transforms for the cameras to the TF tree. The left
    #    camera's TF should be broadcast on the left_cam frame, and the right
    #    camera's TF goes on right_cam.

    br.sendTransform(left_camera)
    br.sendTransform(right_camera)
