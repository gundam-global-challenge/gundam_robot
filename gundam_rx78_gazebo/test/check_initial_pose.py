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

import unittest
import rospy
import time
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
from tf.transformations import euler_from_quaternion


class TestInitialPose(unittest.TestCase):

    @classmethod
    def setUpClass(self):
        rospy.init_node('test_initial_pose', anonymous=True)
        self.base_success = False

    def base_link_cb(self, msg):
        p = msg.pose.pose.position
        r = msg.pose.pose.orientation
        pos = [p.x, p.y, p.z]
        rot = euler_from_quaternion([r.x, r.y, r.z, r.w])

        rospy.loginfo("pos: {:6.3f} {:6.3f} {:6.3f} - ".format(pos[0], pos[1], pos[2]) +
                      "rot: {:6.3f} {:6.3f} {:6.3f}".format(rot[0], rot[1], rot[2]))

        # check position
        if (abs(pos[0]) < 1.0 and abs(pos[1]) < 1.0 and abs(pos[2]) < 1.0 and
                abs(rot[0]) < 0.5 and abs(rot[1]) < 0.5 and abs(rot[2]) < 0.5):
            self.base_success = True

    def test_initial_pose(self):
        rospy.Subscriber("/base_link_ground_truth", Odometry, self.base_link_cb)
        timeout_t = time.time() + 15
        while not rospy.is_shutdown() and not self.base_success and time.time() < timeout_t:
            time.sleep(1.0)
        self.assertTrue(self.base_success)


if __name__ == '__main__':
    import rostest
    rostest.rosrun('gundam_rx78_gazebo', 'test_initial_pose', TestInitialPose)
