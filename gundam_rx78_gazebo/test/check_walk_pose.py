#!/usr/bin/env python

# Copyright (c) 2020 Kei Okada
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

import argparse
import csv
import unittest
import rospy
import actionlib
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion
from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from trajectory_msgs.msg import JointTrajectoryPoint


class TestWalkPose(unittest.TestCase):

    @classmethod
    def setUpClass(self):
        parser = argparse.ArgumentParser(description='Check gundam robot.')
        parser.add_argument('--pos', dest='pos', default=[0, 0, 0], nargs=3, type=float,
                            help='target position')
        parser.add_argument('--rot', dest='rot', default=[0, 0, 0], nargs=3, type=float,
                            help='target orientation')
        parser.add_argument('filename', type=str, nargs='?',
                            help='filename for trajectory pattern csv')
        args, unknown = parser.parse_known_args()
        self.goal_pos = args.pos
        self.goal_rot = args.rot
        self.filename = args.filename

        print("Initializing node... ")
        rospy.init_node('test_walk_pose', anonymous=True)
        self.base_success = False
        rospy.sleep(1)
        print("Running.")

        # copied cdoe from gundam_rx78_control/sample/joint_trajectory_client_csv.py
        # init goal
        goal = FollowJointTrajectoryGoal()
        goal.goal_time_tolerance = rospy.Time(1)

        rospy.loginfo("Opening {}".format(self.filename))
        with open(self.filename) as f:
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
        self.client = actionlib.SimpleActionClient(
            '/fullbody_controller/follow_joint_trajectory',
            FollowJointTrajectoryAction,
        )

        if not self.client.wait_for_server(timeout=rospy.Duration(10)):
            rospy.logerr("Timed out waiting for Action Server")
            rospy.signal_shutdown("Timed out waiting for Action Server")
            sys.exit(1)

        # send goal
        goal.trajectory.header.stamp = rospy.Time.now()
        self.client.send_goal(goal)

    def base_link_cb(self, msg):
        p = msg.pose.pose.position
        r = msg.pose.pose.orientation
        pos = [p.x, p.y, p.z]
        rot = euler_from_quaternion([r.x, r.y, r.z, r.w])

        diff_pos = [abs(pos[0] - self.goal_pos[0]), abs(pos[1] - self.goal_pos[1]), abs(pos[2] - self.goal_pos[2])]
        diff_rot = [abs(rot[0] - self.goal_rot[0]), abs(rot[1] - self.goal_rot[1]), abs(rot[2] - self.goal_rot[2])]

        rospy.loginfo("pos: {:6.3f} {:6.3f} {:6.3f} - ".format(pos[0], pos[1], pos[2]) +
                      "rot: {:6.3f} {:6.3f} {:6.3f} < ".format(rot[0], rot[1], rot[2]) +
                      "pos: {:6.3f} {:6.3f} {:6.3f} - ".format(diff_pos[0], diff_pos[1], diff_pos[2]) +
                      "rot: {:6.3f} {:6.3f} {:6.3f}".format(diff_rot[0], diff_rot[1], diff_rot[2]))

        # check position
        if (diff_pos[0] < 0.5 and diff_pos[1] < 0.5 and diff_pos[2] < 0.5 and
                diff_rot[0] < 0.1 and diff_rot[1] < 0.1 and diff_rot[2] < 0.1):
            self.base_success = True

    def test_walk_pose(self):
        rospy.Subscriber("/base_link_ground_truth", Odometry, self.base_link_cb)
        timeout_t = rospy.Time.now() + rospy.Duration(40)
        while not rospy.is_shutdown() and not self.base_success and rospy.Time.now() < timeout_t:
            rospy.sleep(rospy.Duration(1.0))
        self.assertTrue(self.base_success)


if __name__ == '__main__':
    import rostest
    rostest.rosrun('gundam_rx78_gazebo', 'test_walk_pose', TestWalkPose)
