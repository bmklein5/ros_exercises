#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
import random
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Header


def fake_scan_publisher():
    rospy.init_node('fake_scan_publisher', anonymous=True)

    # define all the params
    pub_topic = rospy.get_param("publish topic", "fake_scan")
    publish_rate = rospy.get_param("publish rate", 20) # 20hz
    a_min = rospy.get_param("angle_min", -2.0 / 3 * np.pi)
    a_max = rospy.get_param("angle_max", 2.0 / 3 * np.pi)
    a_inc = rospy.get_param("angle_increment", 1.0 / 300 * np.pi)
    r_min = rospy.get_param("range_min", 1.0)
    r_max = rospy.get_param("range_max", 10.0)

    pub = rospy.Publisher(pub_topic, LaserScan, queue_size=10)
    rate = rospy.Rate(publish_rate)

    while not rospy.is_shutdown():
        # set up my ranges
        range_list = []
        # print(np.pi)
        # print(aInc)
        seq = range(int(round(abs( (a_max - a_min)/a_inc ))) + 1)
        for i in seq:
            range_list.append(random.uniform(r_min, r_max))

        # define my scan
        scan = LaserScan(
            header = Header(stamp=rospy.get_rostime(), frame_id="base_link"),
            angle_min = a_min,
            angle_max = a_max,
            angle_increment = a_inc,
            range_min = r_min,
            range_max = r_max,
            ranges = range_list
        )

        pub.publish(scan)
        rate.sleep()
        

if __name__ == '__main__':
    try:
        fake_scan_publisher()
    except rospy.ROSInterruptException:
        pass
