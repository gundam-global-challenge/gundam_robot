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

import rospy
import time
import sys
from std_srvs.srv import Empty
from gazebo_msgs.srv import GetPhysicsProperties, GetPhysicsPropertiesResponse


class StartGazeboSimulator:

    def __init__(self):
        self.physics_properties = GetPhysicsPropertiesResponse()
        self.physics_properties.pause = True
        rospy.loginfo(self.physics_properties)
        rospy.wait_for_service('/gazebo/get_physics_properties')
        self.get_physics_properties_service = rospy.ServiceProxy('/gazebo/get_physics_properties', GetPhysicsProperties)
        rospy.wait_for_service('/gazebo/unpause_physics')
        self.start_simulation = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)

    def call_start_simulation(self):
        rospy.logwarn("start gazebo simulation")
        self.start_simulation()
        time.sleep(1.0)  # use wall clock just in case
        self.physics_properties = self.get_physics_properties_service()
        rospy.loginfo(self.physics_properties)


if __name__ == '__main__':
    rospy.init_node('start_simulator', anonymous=True)
    rospy.logwarn("wait for gazebo startup")
    time.sleep(5.0)  # wait for gazebo startup
    start_gazebo_simulator = StartGazeboSimulator()

    # start simulator when pouse is true
    while not rospy.is_shutdown() and start_gazebo_simulator.physics_properties.pause is True:
        try:
            start_gazebo_simulator.call_start_simulation()
        except Exception as e:
            print("Unexpected error:", e)
            pass
