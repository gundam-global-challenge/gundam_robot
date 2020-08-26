#!/usr/bin/env python

# Copyright (c) 2019, 2020 Naoki Hiraoka
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
Reading Trajectory Data from CSV and Send To Robot via Action Client
"""

import rospy

import actionlib

import argparse
import sys
import time
import csv

from control_msgs.msg import (
    FollowJointTrajectoryAction,
    FollowJointTrajectoryGoal,
)
from trajectory_msgs.msg import (
    JointTrajectoryPoint,
)


def main(filename):
    print("Initializing node... ")
    rospy.init_node("joint_trajectory_client_csv_example")
    rospy.sleep(1)
    print("Running. Ctrl-c to quit")

    # init goal
    goal = FollowJointTrajectoryGoal()
    goal.goal_time_tolerance = rospy.Time(1)

    with open(filename) as f:
        reader = csv.reader(f, skipinitialspace=True)
        first_row = True
        for row in reader:
            if first_row:
                goal.trajectory.joint_names = row[1:]
                first_row = False
            else:
                point = JointTrajectoryPoint()
                point.positions = [float(n) for n in row[1:]]
                point.time_from_start = rospy.Duration(float(row[0]))
                goal.trajectory.points.append(point)
    client = actionlib.SimpleActionClient(
        '/fullbody_controller/follow_joint_trajectory',
        FollowJointTrajectoryAction,
    )

    if not client.wait_for_server(timeout=rospy.Duration(10)):
        rospy.logerr("Timed out waiting for Action Server")
        rospy.signal_shutdown("Timed out waiting for Action Server")
        sys.exit(1)

    # send goal
    goal.trajectory.header.stamp = rospy.Time.now()
    client.send_goal(goal)
    print("waiting...")
    if not client.wait_for_result(timeout=rospy.Duration(60)):
        rospy.logerr("Timed out waiting for JTA")
    rospy.loginfo("Exitting...")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Reading CSV trajectory data and send to the robot.')
    parser.add_argument('filename', type=str, nargs=1,
                        help='CSV trajectory data file name')
    args = parser.parse_args(rospy.myargv()[1:])
    main(args.filename[0])
