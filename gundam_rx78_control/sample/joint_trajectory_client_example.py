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
Joint Trajectory Action Client Example
"""

import rospy

import actionlib

import sys
import time

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)


def main():
    print("Initializing node... ")
    rospy.init_node("joint_trajectory_client_example")
    rospy.sleep(1)
    print("Running. Ctrl-c to quit")
    positions = {
        'head_neck_p': 0.0, 'head_neck_y': 0.2,
        'larm_shoulder_p': 0.1, 'larm_shoulder_r':  0.3, 'larm_shoulder_y': -0.1, 'larm_elbow_p': -0.2, 'larm_elbow_p2': -0.2,
        'larm_wrist_r': 0, 'larm_wrist_y': 0, 'larm_gripper': 1.1,
        'rarm_shoulder_p': 0.1, 'rarm_shoulder_r': -0.3, 'rarm_shoulder_y': 0.1, 'rarm_elbow_p': -0.2, 'rarm_elbow_p2': -0.2,
        'rarm_wrist_r': 0, 'rarm_wrist_y': 0, 'rarm_gripper': 1.1,
        'torso_waist_p': -0.05, 'torso_waist_y': 0.0, 'torso_rthrust_p': 0.0, 'torso_rthrust_r': 0.0, 'torso_lthrust_p': 0.0, 'torso_lthrust_r': 0.0,
        'lleg_crotch_p': -0.35, 'lleg_crotch_r':  0.2, 'lleg_crotch_y':   0.35, 'lleg_knee_p':  0.20, 'lleg_knee_p2':  0.20, 'lleg_ankle_p':  0.05, 'lleg_ankle_r': -0.05,
        'rleg_crotch_p':  0.20, 'rleg_crotch_r': -0.1, 'rleg_crotch_y': -0.15, 'rleg_knee_p':  0.05, 'rleg_knee_p2':  0.05, 'rleg_ankle_p': -0.2, 'rleg_ankle_r':  0.1,
    }
    client = actionlib.SimpleActionClient(
        '/fullbody_controller/follow_joint_trajectory',
        FollowJointTrajectoryAction,
    )

    if not client.wait_for_server(timeout=rospy.Duration(10)):
        rospy.logerr("Timed out waiting for Action Server")
        rospy.signal_shutdown("Timed out waiting for Action Server")
        sys.exit(1)

    # init goal
    goal = FollowJointTrajectoryGoal()
    goal.goal_time_tolerance = rospy.Time(1)
    goal.trajectory.joint_names = positions.keys()

    # points
    point = JointTrajectoryPoint()
    goal.trajectory.joint_names = positions.keys()
    point.positions = positions.values()
    point.time_from_start = rospy.Duration(10)
    goal.trajectory.points.append(point)

    point = JointTrajectoryPoint()
    positions['torso_waist_p']   +=  0.2
    positions['torso_waist_y']   +=  0.2
    positions['head_neck_p']      =  0.05
    positions['head_neck_y']      =  0.15
    positions['lleg_crotch_p']   += -0.1
    positions['rleg_crotch_p']   += -0.1
    positions['rarm_shoulder_p'] +=  0.2
    positions['larm_shoulder_p'] +=  0.2
    positions['rarm_elbow_p']    += -0.2
    positions['rarm_elbow_p2']    += -0.2
    positions['larm_elbow_p']    += -0.2
    positions['larm_elbow_p2']    += -0.2
    point.positions  = positions.values()
    point.time_from_start = rospy.Duration(12)
    goal.trajectory.points.append(point)

    # send goal
    goal.trajectory.header.stamp = rospy.Time.now()
    client.send_goal(goal)
    print(goal)
    print("waiting...")
    if not client.wait_for_result(timeout=rospy.Duration(20)):
        rospy.logerr("Timed out waiting for JTA")
    rospy.loginfo("Exitting...")


if __name__ == "__main__":
    main()
