#!/usr/bin/env python

# Copyright (c) 2016, 2019 Kei Okada
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
# 2. Redistributions in binary form must reproduce the above copyright
#    notice, this list of conditions and the following disclaimer in the
#    documentation and/or other materials provided with the distribution.
# 3. Neither the name of the Rethink Robotics nor the names of its
#    contributors may be used to endorse or promote products derived from
#    this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Joint Position Client Example
"""

import rospy
from std_msgs.msg import Float64


def main():
    print("Initializing node... ")
    rospy.init_node("joint_position_client_example")
    positions = {
        'head_neck_p': 0.2, 'head_neck_y': 0.3,
        'larm_shoulder_p': 0.1, 'larm_shoulder_r':  0.3, 'larm_shoulder_y': -0.1, 'larm_elbow_p': -0.2, 'larm_wrist_r': 0, 'larm_wrist_y': 0, 'larm_gripper': -0.01,
        'rarm_shoulder_p': 0.1, 'rarm_shoulder_r': -0.3, 'rarm_shoulder_y': 0.1, 'rarm_elbow_p': -0.2, 'rarm_wrist_r': 0, 'rarm_wrist_y': 0, 'rarm_gripper': 0.01,

        'torso_waist_p': -0.05, 'torso_waist_y': 0.05,

        'lleg_crotch_p': -0.30, 'lleg_crotch_r':  0.2, 'lleg_crotch_y':   0.35, 'lleg_knee_p':  0.20, 'lleg_ankle_p':  0.15, 'lleg_ankle_r': -0.1,
        'rleg_crotch_p':  0.15, 'rleg_crotch_r': -0.1, 'rleg_crotch_y': -0.15, 'rleg_knee_p':  0.05, 'rleg_ankle_p': -0.2, 'rleg_ankle_r':  0.1,
    }

    pub = {}
    for name, value in positions.items():
        pub[name] = rospy.Publisher(
            name + '_position/command', Float64, queue_size=1)

    rospy.sleep(1)  # wait for publisher started...
    for name, value in positions.items():
        print ("send %f to %s_position/command" % (value, name))
        pub[name].publish(Float64(data=value))

    rospy.sleep(2)  # wait for publisher started...
    rospy.loginfo("Exitting...")


if __name__ == "__main__":
    main()
