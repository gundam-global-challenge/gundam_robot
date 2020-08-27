#!/usr/bin/env python

# Copyright (c) 2016, 2017, 2018 Kei Okada
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
# 3. Neither the name of the Kei Okada nor the names of its
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

from collada import *  # Do not use "pip install pycollada". Use "apt install python-collada".
from urdf_parser_py.urdf import *
from tf.transformations import *
import copy
import os
import sys
import math
import numpy
import argparse
import trimesh  # Use "pip install --user trimesh".
from simplify_collada import simplify_collada
from mergenode_collada import mergenode_collada
from scale_collada import scale_collada
from scipy.spatial.transform import Rotation  # Do not use "apt install python-scipy". Use "pip install --user scipy==1.2.2".
# xmlutil.COLLADA_NS = 'http://www.collada.org/2008/03/COLLADASchema'

depth_ = 0
scale_ = 0.1  # original file uses cm unit
density = 1.22e2
all_weight_ = 0.0
root_offset = numpy.array([[0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 0, 1]], dtype=numpy.float32)  # original file is Y_UP

zero_pid = {'p': 0.0, 'i': 0.0, 'd': 0.0}
default_pid = {'p': 10000000.0, 'i': 500.0, 'd': 200000.0}
head_pid = {'p': 1000000.0, 'i': 500.0, 'd': 200000.0}
torso_pid = {'p': 20000000.0, 'i': 10000.0, 'd': 1000000.0}
elbow_p_pid = {'p': 20000000.0, 'i': 1000.0, 'd': 40000.0}
wrist_pid = {'p': 100000.0, 'i': 50.0, 'd': 20000.0}
gripper_pid = {'p': 100000.0, 'i': 100.0, 'd': 100.0}
finger_pid = {'p': 100000.000000, 'i': 100.000000, 'd': 100.000000}
crotch_p_pid = {'p': 400000000.0, 'i': 4000000.0, 'd': 500000.0}
crotch_r_pid = {'p': 200000000.0, 'i': 1000000.0, 'd': 500000.0}
crotch_y_pid = {'p': 200000000.0, 'i': 1000000.0, 'd': 500000.0}
knee_p_pid = {'p': 200000000.0, 'i': 1000000.0, 'd': 100000.0}
knee_p_mimic_pid = {'p': 200000000.0, 'i': 1000000.0, 'd': 100000.0}
ankle_pid = {'p': 100000000.0, 'i': 5000.0, 'd': 500000.0}
ankle_r_mimic_pid = {'p': 100000000.0, 'i': 1000000.0, 'd': 50000.0}
ankle_p_mimic_pid = {'p': 500000.0, 'i': 500.0, 'd': 50000.0}
cover_pid = {'p': 50000.0, 'i': 500.0, 'd': 5000.0}
thrust_pid = {'p': 10000000.0, 'i': 500.0, 'd': 20000.0}

joints_ = [
    # axis [pitch, roll, yaw]
    # backpack
    ['rx78_Null_013', {'joint_type': 'fixed'}],
    ['rx78_Null_012', {'joint_type': 'fixed'}],  # sword
    ['rx78_Null_011', {'joint_type': 'fixed'}],
    ['rx78_Null_010', {'joint_type': 'fixed'}],
    ['rx78_Null_009', {'joint_type': 'fixed'}],  # sword
    ['rx78_Null_008', {'joint_type': 'fixed'}],
    ['rx78_Null_007', {'joint_type': 'fixed'}],
    ['rx78_Null_005', {'name': 'torso_rthrust_p', 'axis': [1, 0, 0], 'limit_lower': -math.pi / 4, 'limit_upper': math.pi / 2, 'pid': thrust_pid}],  # jet
    ['rx78_Null_005', {'name': 'torso_rthrust_r', 'axis': [0, 1, 0], 'limit_lower': -math.pi / 4, 'limit_upper': math.pi / 4, 'pid': thrust_pid}],
    ['rx78_Null_006', {'name': 'torso_lthrust_p', 'axis': [1, 0, 0], 'limit_lower': -math.pi / 4, 'limit_upper': math.pi / 2, 'pid': thrust_pid}],
    ['rx78_Null_006', {'name': 'torso_lthrust_r', 'axis': [0, 1, 0], 'limit_lower': -math.pi / 4, 'limit_upper': math.pi / 4, 'pid': thrust_pid}],
    ['rx78_Null_082', {'joint_type': 'fixed'}],  # skip torso parts, which is also has another joint
    ['rx78_Null_083', {'joint_type': 'fixed'}],  # torso?

    # torso
    ['rx78_Null_018', {'name': 'torso_waist_y', 'axis': [0, 0, 1], 'limit_lower': -math.pi / 12, 'limit_upper': math.pi / 12, 'pid': torso_pid}],
    ['rx78_Null_017', {'name': 'torso_waist_p', 'axis': [1, 0, 0], 'limit_lower': -math.pi / 4, 'limit_upper': math.pi / 4, 'pid': torso_pid}],
    ['rx78_Null_016', {'name': 'torso_waist_p2', 'axis': [1, 0, 0], 'limit_lower': -math.pi / 4, 'limit_upper': math.pi / 4, 'axis': [1, 0, 0], 'pid': torso_pid}],

    # head
    ['rx78_Null_015', {'name': 'head_neck_y', 'axis': [0, 0, 1], 'limit_lower': -math.pi / 2, 'limit_upper': math.pi / 2, 'pid': head_pid}],
    ['rx78_Null_014', {'name': 'head_neck_p', 'axis': [1, 0, 0], 'limit_lower': -math.pi / 5, 'limit_upper': math.pi / 4, 'pid': head_pid}],

    # larm
    ['rx78_Null_004', {'name': 'larm_shoulder_p', 'axis': [1, 0, 0], 'limit_lower': -math.pi, 'limit_upper': math.pi / 2}],
    ['rx78_Null_001', {'name': 'larm_shoulder_r', 'axis': [0, -1, 0], 'limit_lower': -math.pi / 6, 'limit_upper': math.pi / 2}],
    ['rx78_Null_002', {'name': 'larm_shoulder_y', 'axis': [0, 0, 1], 'limit_lower': -math.pi / 2, 'limit_upper': math.pi / 2}],
    ['rx78_Null_003', {'name': 'larm_elbow_p', 'axis': [1, 0, 0], 'limit_lower': -math.pi / 3, 'limit_upper': math.pi / 24, 'pid': elbow_p_pid}],
    ['rx78_Null_066', {'name': 'larm_elbow_p2', 'axis': [1, 0, 0], 'limit_lower': -math.pi / 3, 'limit_upper': math.pi / 24, 'pid': elbow_p_pid}],
    ['rx78_Null_067', {'name': 'larm_wrist_y', 'axis': [0, 0, 1], 'limit_lower': -math.pi / 2, 'limit_upper': math.pi / 2, 'pid': wrist_pid}],
    ['rx78_Null_069', {'name': 'larm_wrist_r', 'axis': [0, -1, 0], 'limit_lower': -math.pi / 2, 'limit_upper': math.pi / 2, 'pid': wrist_pid}],
    ['rx78_Null_048', {'joint_type': 'fixed'}],  # shoulder-p cover
    ['rx78_Null_065', {'joint_type': 'fixed'}],  # elbow-p internal
    # left hand
    ['rx78_Null_059', {'name': 'larm_gripper_index0_mimic', 'axis': [0, 1, 0], 'limit_lower': -math.pi / 24, 'limit_upper': math.pi / 2, 'mimic': 'larm_gripper', 'mimic_multiplier': 1.0, 'pid': finger_pid}],  # index
    ['rx78_Null_060', {'name': 'larm_gripper_index1_mimic', 'axis': [0, 1, 0], 'limit_lower': -math.pi / 24, 'limit_upper': math.pi / 2, 'mimic': 'larm_gripper', 'mimic_multiplier': 1.0, 'pid': finger_pid}],
    ['rx78_Null_061', {'name': 'larm_gripper_index2_mimic', 'axis': [0, 1, 0], 'limit_lower': -math.pi / 24, 'limit_upper': math.pi / 2, 'mimic': 'larm_gripper', 'mimic_multiplier': 1.0, 'pid': finger_pid}],
    ['rx78_Null_062', {'name': 'larm_gripper', 'axis': [0, 1, 0], 'limit_lower': -math.pi / 24, 'limit_upper': math.pi / 2, 'pid': gripper_pid}],  # thumb
    ['rx78_Null_063', {'name': 'larm_gripper_thumb1_mimic', 'axis': [1, 0, 0], 'limit_lower': -math.pi / 5, 'limit_upper': math.pi / 4,
                       'mimic': 'larm_gripper', 'mimic_multiplier': 1.0, 'mimic_offset': -math.pi / 4, 'pid': finger_pid}],
    ['rx78_Null_064', {'name': 'larm_gripper_thumb2_mimic', 'axis': [1, 0, 0], 'limit_lower': -math.pi / 24, 'limit_upper': math.pi / 2, 'mimic': 'larm_gripper', 'mimic_multiplier': 1.0, 'pid': finger_pid}],
    ['rx78_Null_068', {'name': 'larm_gripper_middle0_mimic', 'axis': [0, 1, 0], 'limit_lower': -math.pi / 24, 'limit_upper': math.pi / 2, 'mimic': 'larm_gripper', 'mimic_multiplier': 1.0, 'pid': finger_pid}],
    ['rx78_Null_070', {'name': 'larm_gripper_middle1_mimic', 'axis': [0, 1, 0], 'limit_lower': -math.pi / 24, 'limit_upper': math.pi / 2, 'mimic': 'larm_gripper', 'mimic_multiplier': 1.0, 'pid': finger_pid}],
    ['rx78_Null_071', {'name': 'larm_gripper_middle2_mimic', 'axis': [0, 1, 0], 'limit_lower': -math.pi / 24, 'limit_upper': math.pi / 2, 'mimic': 'larm_gripper', 'mimic_multiplier': 1.0, 'pid': finger_pid}],
    ['rx78_Null_072', {'name': 'larm_gripper_ring0_mimic', 'axis': [0, 1, 0], 'limit_lower': -math.pi / 24, 'limit_upper': math.pi / 2, 'mimic': 'larm_gripper', 'mimic_multiplier': 1.0, 'pid': finger_pid}],
    ['rx78_Null_073', {'name': 'larm_gripper_ring1_mimic', 'axis': [0, 1, 0], 'limit_lower': -math.pi / 24, 'limit_upper': math.pi / 2, 'mimic': 'larm_gripper', 'mimic_multiplier': 1.0, 'pid': finger_pid}],
    ['rx78_Null_074', {'name': 'larm_gripper_ring2_mimic', 'axis': [0, 1, 0], 'limit_lower': -math.pi / 24, 'limit_upper': math.pi / 2, 'mimic': 'larm_gripper', 'mimic_multiplier': 1.0, 'pid': finger_pid}],
    ['rx78_Null_075', {'name': 'larm_gripper_little0_mimic', 'axis': [0, 1, 0], 'limit_lower': -math.pi / 24, 'limit_upper': math.pi / 2, 'mimic': 'larm_gripper', 'mimic_multiplier': 1.0, 'pid': finger_pid}],
    ['rx78_Null_076', {'name': 'larm_gripper_little1_mimic', 'axis': [0, 1, 0], 'limit_lower': -math.pi / 24, 'limit_upper': math.pi / 2, 'mimic': 'larm_gripper', 'mimic_multiplier': 1.0, 'pid': finger_pid}],
    ['rx78_Null_077', {'name': 'larm_gripper_little2_mimic', 'axis': [0, 1, 0], 'limit_lower': -math.pi / 24, 'limit_upper': math.pi / 2, 'mimic': 'larm_gripper', 'mimic_multiplier': 1.0, 'pid': finger_pid}],

    # rarm
    ['rx78_Null_049', {'name': 'rarm_shoulder_p', 'axis': [1, 0, 0], 'limit_lower': -math.pi, 'limit_upper': math.pi / 2}],
    ['rx78_Null_050', {'name': 'rarm_shoulder_r', 'axis': [0, -1, 0], 'limit_lower': -math.pi / 2, 'limit_upper': math.pi / 6}],
    ['rx78_Null_051', {'name': 'rarm_shoulder_y', 'axis': [0, 0, 1], 'limit_lower': -math.pi / 2, 'limit_upper': math.pi / 2}],
    ['rx78_Null_052', {'name': 'rarm_elbow_p', 'axis': [1, 0, 0], 'limit_lower': -math.pi / 3, 'limit_upper': math.pi / 24, 'pid': elbow_p_pid}],
    ['rx78_Null_053', {'name': 'rarm_elbow_p2', 'axis': [1, 0, 0], 'limit_lower': -math.pi / 3, 'limit_upper': math.pi / 24, 'pid': elbow_p_pid}],
    ['rx78_Null_054', {'name': 'rarm_wrist_y', 'axis': [0, 0, 1], 'limit_lower': -math.pi / 2, 'limit_upper': math.pi / 2, 'pid': wrist_pid}],
    ['rx78_Null_055', {'name': 'rarm_wrist_r', 'axis': [0, -1, 0], 'limit_lower': -math.pi / 2, 'limit_upper': math.pi / 2, 'pid': wrist_pid}],
    ['rx78_Null_081', {'joint_type': 'fixed'}],  # shoulder-p cover
    # right hand
    ['rx78_Null_021', {'name': 'rarm_gripper_middle0_mimic', 'axis': [0, -1, 0], 'limit_lower': -math.pi / 24, 'limit_upper': math.pi / 2, 'mimic': 'rarm_gripper', 'mimic_multiplier': 1.0, 'pid': finger_pid}],
    ['rx78_Null_022', {'name': 'rarm_gripper_middle1_mimic', 'axis': [0, -1, 0], 'limit_lower': -math.pi / 24, 'limit_upper': math.pi / 2, 'mimic': 'rarm_gripper', 'mimic_multiplier': 1.0, 'pid': finger_pid}],
    ['rx78_Null_023', {'name': 'rarm_gripper_middle2_mimic', 'axis': [0, -1, 0], 'limit_lower': -math.pi / 24, 'limit_upper': math.pi / 2, 'mimic': 'rarm_gripper', 'mimic_multiplier': 1.0, 'pid': finger_pid}],
    ['rx78_Null_024', {'name': 'rarm_gripper_index0_mimic', 'axis': [0, -1, 0], 'limit_lower': -math.pi / 24, 'limit_upper': math.pi / 2, 'mimic': 'rarm_gripper', 'mimic_multiplier': 1.0, 'pid': finger_pid}],  # index
    ['rx78_Null_025', {'name': 'rarm_gripper_index1_mimic', 'axis': [0, -1, 0], 'limit_lower': -math.pi / 24, 'limit_upper': math.pi / 2, 'mimic': 'rarm_gripper', 'mimic_multiplier': 1.0, 'pid': finger_pid}],
    ['rx78_Null_026', {'name': 'rarm_gripper_index2_mimic', 'axis': [0, -1, 0], 'limit_lower': -math.pi / 24, 'limit_upper': math.pi / 2, 'mimic': 'rarm_gripper', 'mimic_multiplier': 1.0, 'pid': finger_pid}],
    ['rx78_Null_027', {'name': 'rarm_gripper_little0_mimic', 'axis': [0, -1, 0], 'limit_lower': -math.pi / 24, 'limit_upper': math.pi / 2, 'mimic': 'rarm_gripper', 'mimic_multiplier': 1.0, 'pid': finger_pid}],
    ['rx78_Null_028', {'name': 'rarm_gripper_little1_mimic', 'axis': [0, -1, 0], 'limit_lower': -math.pi / 24, 'limit_upper': math.pi / 2, 'mimic': 'rarm_gripper', 'mimic_multiplier': 1.0, 'pid': finger_pid}],
    ['rx78_Null_029', {'name': 'rarm_gripper_little2_mimic', 'axis': [0, -1, 0], 'limit_lower': -math.pi / 24, 'limit_upper': math.pi / 2, 'mimic': 'rarm_gripper', 'mimic_multiplier': 1.0, 'pid': finger_pid}],
    ['rx78_Null_056', {'name': 'rarm_gripper', 'axis': [0, -1, 0], 'limit_lower': -math.pi / 24, 'limit_upper': math.pi / 2, 'pid': gripper_pid}],  # thumb
    ['rx78_Null_057', {'name': 'rarm_gripper_thumb1_mimic', 'axis': [1, 0, 0], 'limit_lower': -math.pi / 5, 'limit_upper': math.pi / 4,
                       'mimic': 'rarm_gripper', 'mimic_multiplier': 1.0, 'mimic_offset': -math.pi / 4, 'pid': finger_pid}],
    ['rx78_Null_058', {'name': 'rarm_gripper_thumb2_mimic', 'axis': [1, 0, 0], 'limit_lower': -math.pi / 24, 'limit_upper': math.pi / 2, 'mimic': 'rarm_gripper', 'mimic_multiplier': 1.0, 'pid': finger_pid}],
    ['rx78_Null_078', {'name': 'rarm_gripper_ring0_mimic', 'axis': [0, -1, 0], 'limit_lower': -math.pi / 24, 'limit_upper': math.pi / 2, 'mimic': 'rarm_gripper', 'mimic_multiplier': 1.0, 'pid': finger_pid}],
    ['rx78_Null_079', {'name': 'rarm_gripper_ring1_mimic', 'axis': [0, -1, 0], 'limit_lower': -math.pi / 24, 'limit_upper': math.pi / 2, 'mimic': 'rarm_gripper', 'mimic_multiplier': 1.0, 'pid': finger_pid}],
    ['rx78_Null_080', {'name': 'rarm_gripper_ring2_mimic', 'axis': [0, -1, 0], 'limit_lower': -math.pi / 24, 'limit_upper': math.pi / 2, 'mimic': 'rarm_gripper', 'mimic_multiplier': 1.0, 'pid': finger_pid}],

    # lleg
    ['rx78_Null_033', {'name': 'lleg_crotch_p_back_mimic', 'axis': [1, 0, 0], 'limit_lower': -math.pi / 4, 'limit_upper': math.pi / 6,
                       'mimic': 'lleg_crotch_p', 'mimic_multiplier': 0.5, 'pid': cover_pid}],  # back cover
    ['rx78_Null_034', {'name': 'lleg_crotch_p_front_mimic', 'axis': [1, 0, 0], 'limit_lower': -math.pi / 4, 'limit_upper': math.pi / 6,
                       'mimic': 'lleg_crotch_p', 'mimic_multiplier': 0.5, 'pid': cover_pid}],  # front cover
    ['rx78_Null_047', {'name': 'lleg_crotch_r_mimic', 'axis': [0, -1, 0], 'limit_lower': -math.pi / 4 * 0.6, 'limit_upper': math.pi / 9 * 0.6,
                       'mimic': 'lleg_crotch_r', 'mimic_multiplier': -0.6, 'axis': [0, 1, 0], 'pid': cover_pid}],  # side cover
    ['rx78_Null_035', {'name': 'lleg_crotch_p', 'axis': [1, 0, 0], 'limit_lower': -math.pi / 2, 'limit_upper': math.pi / 3, 'pid': crotch_p_pid}],
    ['rx78_Null_035', {'name': 'lleg_crotch_r', 'joint_type': 'revolute', 'axis': [0, -1, 0], 'limit_lower': -math.pi / 9, 'limit_upper': math.pi / 4, 'pid': crotch_r_pid}],  # hack for crotch-r _035->_029->_036
    ['rx78_Null_036', {'name': 'lleg_crotch_y', 'axis': [0, 0, 1], 'limit_lower': -math.pi / 4, 'limit_upper': math.pi / 4, 'pid': crotch_y_pid}],
    ['rx78_Null_037', {'name': 'lleg_knee_p', 'axis': [1, 0, 0], 'limit_lower': -math.pi / 24, 'limit_upper': math.pi / 3, 'pid': knee_p_pid}],
    ['rx78_Null_038', {'name': 'lleg_knee_p2', 'axis': [1, 0, 0], 'limit_lower': -math.pi / 24, 'limit_upper': math.pi / 3, 'pid': knee_p_mimic_pid}],
    ['rx78_Null_039', {'name': 'lleg_ankle_p', 'axis': [1, 0, 0], 'limit_lower': -math.pi / 3, 'limit_upper': math.pi / 3, 'pid': ankle_pid}],
    ['rx78_Null_041', {'name': 'lleg_ankle_r', 'axis': [-1, 0, 0], 'limit_lower': -math.pi / 6, 'limit_upper': math.pi / 6, 'pid': ankle_pid}],

    ['rx78_Null_040', {'name': 'lleg_ankle_r_mimic', 'axis': [-1, 0, 0], 'limit_lower': -math.pi / 6, 'limit_upper': math.pi / 6,
                       'mimic': 'lleg_ankle_r', 'mimic_multiplier': 1.0, 'pid': ankle_r_mimic_pid}],  # ankle back
    ['rx78_Null_042', {'joint_type': 'fixed'}],  # sole
    ['rx78_Null_043', {'joint_type': 'fixed'}],  # sole
    ['rx78_Null_044', {'joint_type': 'fixed'}],  # sole
    ['rx78_Null_045', {'joint_type': 'fixed'}],  # sole
    ['rx78_Null_046', {'name': 'lleg_ankle_p_mimic', 'axis': [1, 0, 0], 'limit_lower': -math.pi / 6, 'limit_upper': math.pi / 6,
                       'mimic': 'lleg_ankle_p', 'mimic_multiplier': -0.5, 'pid': ankle_p_mimic_pid}],  # ankle cover

    # rleg
    ['rx78_Null_084', {'name': 'rleg_crotch_p_front_mimic', 'axis': [1, 0, 0], 'limit_lower': -math.pi / 4, 'limit_upper': math.pi / 6,
                       'mimic': 'rleg_crotch_p', 'mimic_multiplier': 0.5, 'pid': cover_pid}],  # front cover
    ['rx78_Null_031', {'name': 'rleg_crotch_p_back_mimic', 'axis': [1, 0, 0], 'limit_lower': -math.pi / 4, 'limit_upper': math.pi / 6,
                       'mimic': 'rleg_crotch_p', 'mimic_multiplier': 0.5, 'pid': cover_pid}],  # back cover
    ['rx78_Null_032', {'name': 'rleg_crotch_r_mimic', 'axis': [0, -1, 0], 'limit_lower': -math.pi / 4 * 0.6, 'limit_upper': math.pi / 9 * 0.6,
                       'mimic': 'rleg_crotch_r', 'mimic_multiplier': 0.6, 'pid': cover_pid}],  # side cover
    ['rx78_Null_085', {'name': 'rleg_crotch_p', 'axis': [1, 0, 0], 'limit_lower': -math.pi / 2, 'limit_upper': math.pi / 3, 'pid': crotch_p_pid}],
    ['rx78_Null_085', {'name': 'rleg_crotch_r', 'axis': [0, -1, 0], 'limit_lower': -math.pi / 4, 'limit_upper': math.pi / 9, 'pid': crotch_r_pid}],  # hack for crotch-r _085->_086->_086
    ['rx78_Null_086', {'name': 'rleg_crotch_y', 'axis': [0, 0, 1], 'limit_lower': -math.pi / 4, 'limit_upper': math.pi / 4, 'pid': crotch_y_pid}],
    ['rx78_Null_087', {'name': 'rleg_knee_p', 'axis': [1, 0, 0], 'limit_lower': -math.pi / 24, 'limit_upper': math.pi / 3, 'pid': knee_p_pid}],
    ['rx78_Null_088', {'name': 'rleg_knee_p2', 'axis': [1, 0, 0], 'limit_lower': -math.pi / 24, 'limit_upper': math.pi / 3, 'pid': knee_p_mimic_pid}],
    ['rx78_Null_089', {'name': 'rleg_ankle_p', 'axis': [1, 0, 0], 'limit_lower': -math.pi / 3, 'limit_upper': math.pi / 3, 'pid': ankle_pid}],
    ['rx78_Null_091', {'name': 'rleg_ankle_r', 'axis': [-1, 0, 0], 'limit_lower': -math.pi / 6, 'limit_upper': math.pi / 6, 'pid': ankle_pid}],

    ['rx78_Null_090', {'name': 'rleg_ankle_r_mimic', 'axis': [-1, 0, 0], 'limit_lower': -math.pi / 6, 'limit_upper': math.pi / 6,
                       'mimic': 'rleg_ankle_r', 'mimic_multiplier': 1.0, 'pid': ankle_r_mimic_pid}],  # ankle back
    ['rx78_Null_092', {'joint_type': 'fixed'}],  # sole
    ['rx78_Null_093', {'joint_type': 'fixed'}],  # sole
    ['rx78_Null_094', {'joint_type': 'fixed'}],  # sole
    ['rx78_Null_095', {'joint_type': 'fixed'}],  # sole
    ['rx78_Null_030', {'name': 'rleg_ankle_p_mimic', 'axis': [1, 0, 0], 'limit_lower': -math.pi / 6, 'limit_upper': math.pi / 6,
                       'mimic': 'rleg_ankle_p', 'mimic_multiplier': -0.5, 'pid': ankle_p_mimic_pid}],  # ankle cover

]


def get_bouding_box(geometries):
    bbox_min = []
    bbox_max = []
    for g in geometries:
        for p in g.primitives:
            if p.vertex is not None:
                for i in p.vertex_index:
                    bbox_min.append(numpy.amin(p.vertex[i], axis=0))
                    bbox_max.append(numpy.amax(p.vertex[i], axis=0))

    xyz = list((numpy.amax(numpy.array(bbox_max), axis=0) +
                numpy.amin(numpy.array(bbox_min), axis=0)) / 2)
    size = list(numpy.amax(numpy.array(bbox_max), axis=0) -
                numpy.amin(numpy.array(bbox_min), axis=0))
    return Collision(
        origin=Pose(xyz=xyz),
        geometry=Box(size=size))


def calc_inertia(collision, density=100.0):
    global all_weight_
    bbox_x = collision.geometry.size[0]
    bbox_y = collision.geometry.size[1]
    bbox_z = collision.geometry.size[2]
    mass = density * bbox_x * bbox_y * bbox_z
    # GGC hack, small mass reduce stability of simulation
    if 0 < mass and mass < 50:
        mass = 50
    all_weight_ += mass
    return Inertial(mass=mass,
                    origin=copy.deepcopy(collision.origin),
                    inertia=Inertia(
                        ixx=mass * (bbox_y * bbox_y + bbox_z * bbox_z) / 12.0,
                        iyy=mass * (bbox_z * bbox_z + bbox_x * bbox_x) / 12.0,
                        izz=mass * (bbox_x * bbox_x + bbox_y * bbox_y) / 12.0))


def get_volume(geometries, density):
    global all_weight_
    if len(geometries) > 0:
        m = 0.0
        c = numpy.array([0.0, 0.0, 0.0])
        I = numpy.array([[0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]])
        for g in geometries:
            for p in g.primitives:
                _mesh = trimesh.Trimesh(vertices=p.vertex.astype("float64"), faces=p.vertex_index, face_normals=p.normal[p.normal_index].astype("float64"), validate=True).convex_hull
                _m = _mesh.volume * density
                _c = _mesh.center_mass
                _I = _mesh.moment_inertia * density
                m += _m
                c += _m * _c
                I += _I + _m * (numpy.inner(_c, _c) * numpy.identity(3) - numpy.outer(_c, _c))
        if m > 0:
            all_weight_ += m
            c = c / m
            I = I - m * (numpy.inner(c, c) * numpy.identity(3) - numpy.outer(c, c))
            if m < 50:  # GGC hack, small mass reduce stability of simulation
                I *= 50 / m
                m = 50
            components, vectors = trimesh.inertia.principal_axis(I)
            vectors[2] = numpy.cross(vectors[0], vectors[1])
            rotate = Rotation.from_dcm(vectors.transpose()).as_euler('xyz', degrees=False)
            return Inertial(mass=m,
                            origin=Pose(xyz=c, rpy=[rotate[0], rotate[1], rotate[2]]),
                            inertia=Inertia(
                                ixx=components[0],
                                iyy=components[1],
                                izz=components[2]))
        else:
            return None
    else:
        return None


def retrive_node(nodes, joints_dict, links_dict, parent=None):
    global robot_, depth_
    # if len(robot_.joints) > 8: return True ############################# FOR
    # DEBUG
    depth_ += 1
    for node in nodes:
        print(' ' * depth_),  # write indent
        try:
            print('id .. {} {}'.format(node.id, type(node)))
        except:
            print('node .. {}'.format(node))
        if isinstance(node, scene.Node):
            if node.id:
                if node.id[-5:] == '_link':
                    linkname = node.id
                else:
                    linkname = node.id + '_link'
                l = Link(name=linkname,
                         visual=None,
                         inertial=None,
                         collision=None,
                         origin=None)
                # add link
                robot_.add_link(l)
                if parent:
                    if parent.id[-5:] == '_link':
                        parentname = parent.id
                    else:
                        parentname = parent.id + '_link'
                    j = Joint(name=node.id + '_joint',
                              parent=parentname,
                              child=l.name,
                              joint_type='fixed',
                              origin=Pose(
                                  xyz=translation_from_matrix(node.matrix),
                                  rpy=euler_from_matrix(node.matrix))
                              )
                    print(
                        '{}  - {}'.format(' ' * depth_, [type(n) for n in node.children]))
                    # if not any(isinstance(n, scene.GeometryNode) for n in node.children):
                    #     j.joint_type = 'revolute'
                    #     # j.axis = numpy.array(parent.matrix)[:3, :3].dot([0,
                    #     # 0, 1])
                    #     j.axis = [0, 0, 1]
                    #     if len(node.transforms) > 1:
                    #         print(node.transforms[1:])
                    #     if any(isinstance(n, scene.RotateTransform) for n in node.transforms):
                    #         j.axis = [1, 0, 0]
                    #     j.limit = JointLimit(
                    #         lower=-math.pi / 2, upper=math.pi / 2, effort=1000000000, velocity=1000000)
                    #
                    if node.id in joints_dict:
                        j.joint_type = 'revolute'
                        j.axis = [0, 0, 1]
                        if len(node.transforms) > 1:
                            print(node.transforms[1:])
                        if any(isinstance(n, scene.RotateTransform) for n in node.transforms):
                            j.axis = [1, 0, 0]
                        j.limit = JointLimit(
                            lower=-math.pi / 2, upper=math.pi / 2, effort=1000000000, velocity=1000000)
                        j.dynamics = JointDynamics(
                            damping='3e2', friction='1e3')
                        if 'joint_type' in joints_dict[node.id]:
                            j.joint_type = joints_dict[node.id]['joint_type']
                            if j.joint_type == 'fixed':
                                j.limit = None
                                j.axis = None
                                j.dynamics = None
                        if 'name' in joints_dict[node.id]:
                            j.name = joints_dict[node.id]['name']
                        if 'axis' in joints_dict[node.id]:
                            j.axis = joints_dict[node.id]['axis']
                        if 'limit_lower' in joints_dict[node.id]:
                            j.limit.lower = joints_dict[node.id]['limit_lower']
                        if 'limit_upper' in joints_dict[node.id]:
                            j.limit.upper = joints_dict[node.id]['limit_upper']
                        if 'child' in joints_dict[node.id]:
                            j.child = joints_dict[node.id]['child'] + '_link'
                        if 'origin_xyz' in joints_dict[node.id]:
                            j.origin.xyz = joints_dict[node.id]['origin_xyz']
                        if 'origin_rpy' in joints_dict[node.id]:
                            j.origin.rpy = joints_dict[node.id]['origin_rpy']
                        if args.no_mimic and 'mimic' in joints_dict[node.id]:
                            # disable mimic
                            j.joint_type = 'fixed'
                            j.limit = None
                            j.axis = None
                        else:
                            if 'mimic' in joints_dict[node.id]:
                                j.mimic = JointMimic(
                                    joint_name=joints_dict[node.id]['mimic'])
                            if 'mimic_multiplier' in joints_dict[node.id]:
                                j.mimic.multiplier = joints_dict[
                                    node.id]['mimic_multiplier']
                            if 'mimic_offset' in joints_dict[node.id]:
                                j.mimic.offset = joints_dict[
                                    node.id]['mimic_offset']
                    #
                    # add joint
                    robot_.add_joint(j)

            retrive_node(node.children, joints_dict, links_dict, node)
            depth_ -= 1
        elif isinstance(node, scene.GeometryNode):
            # print('writing mesh file to meshes/{}.dae'.format(node.geometry.id))
            # write mesh
            c = Collada()
            g = node.geometry
            n = scene.Node(g.name + '-node', [node])
            s = scene.Scene(g.name + '-scene', [])
            # s.nodes.extend(parent.transforms) # ?? need this?
            s.nodes.append(n)
            cont = asset.Contributor(
                author="Association GUNDAM GLOBAL CHALLENGE",
                comments="This file is automatically generated by " +
                (' '.join(sys.argv)).replace('--', '\-\-') + ' '
                "and distributed under the TERMS OF USE FOR GUNDAM RESEARCH OPEN SIMULATOR Attribution-NonCommercial-ShareAlike",
                copyright="SOTSU, SUNRISE / GUNDAM GLOBAL CHALLENGE",
            )
            c.assetInfo.contributors.append(cont)
            c.geometries.append(g)
            c.materials = [m.target for m in node.materials]
            c.effects = [m.target.effect for m in node.materials]
            c.scenes.append(s)
            c.scene = s
            if args.write_mesh:
                c.write('meshes/{}.dae'.format(g.id))
            #
            # update urdf
            l = [l for l in robot_.links if l.name == parent.id + '_link'][0]
            l.visuals.append(Visual(
                geometry=Mesh(
                    filename='package://gundam_rx78_description/meshes/{}.dae'.format(
                        g.id))))
            # get bounding box of scene
            l.collisions.append(Collision(
                geometry=Mesh(
                    filename='package://gundam_rx78_description/meshes/{}.dae'.format(
                        g.id))))
            if l.name in link_dict:
                link_dict[l.name].append(g)
            else:
                link_dict[l.name] = [g]
            l.inertial = None
        elif isinstance(node, scene.ExtraNode):
            pass
        else:
            print('skipping {}'.format(node))


def add_gazebo_nodes(robot, link_dict):
    for j in robot.joints:
        if j.joint_type != "revolute":
            continue

        # actuated joint
        if not args.no_mimic or j.mimic is None:
            # add extra info for actuated joints
            g = etree.Element('gazebo', reference=j.name)
            etree.SubElement(g, 'provideFeedback').text = '1'
            etree.SubElement(g, 'implicitSpringDamper').text = '0'
            robot.add_aggregate('gazebo', g)
            # add transmission
            trans = Transmission()
            trans.name = j.name + '_trans'
            joi = TransmissionJoint(j.name)
            if args.controller_type == 'position':
                joi.add_aggregate(
                    'hardwareInterface', 'hardware_interface/PositionJointInterface')
            if args.controller_type == 'velocity':
                joi.add_aggregate(
                    'hardwareInterface', 'hardware_interface/VelocityJointInterface')
            if args.controller_type == 'effort':
                joi.add_aggregate(
                    'hardwareInterface', 'hardware_interface/EffortJointInterface')
            act = Actuator(j.name + '_motor')
            act.mechanicalReduction = 1.0
            trans.add_aggregate('joint', joi)
            trans.add_aggregate('actuator', act)
            trans.type = "transmission_interface/SimpleTransmission"
            robot.add_aggregate('transmission', trans)

    for l in robot.links:
        g = etree.Element('gazebo', reference=l.name)
        etree.SubElement(g, 'selfCollide').text = 'false'
        # etree.SubElement(g, 'mu1').text = '0.2'
        # etree.SubElement(g, 'mu2').text = '0.2'
        etree.SubElement(g, 'mu1').text = '1.5'
        etree.SubElement(g, 'mu2').text = '1.5'
        etree.SubElement(g, 'mu2').text = '9000'
        etree.SubElement(g, 'kp').text = '140000000.0'
        etree.SubElement(g, 'kd').text = '280000.0'
        etree.SubElement(g, 'fdir1').text = '1 0 0'
        etree.SubElement(g, 'maxVel').text = '10.0'
        robot.add_aggregate('gazebo', g)

        # calculate mass property
        if l.name != "base_link":
            if "addition_null" in l.name:  # avoid null link ** GGC HACK
                l.collision = Collision(origin=Pose(xyz=[0, 0, 0]), geometry=Box(size=[0.8, 0.8, 0.8]))
                l.inertial = calc_inertia(l.collision, 400)
            else:
                if l.inertial is None:
                    if l.name in link_dict:
                        l.inertial = get_volume(link_dict[l.name], density)

    # for urdfdom_py = 0.4.0
    # Once https://github.com/ros/urdf_parser_py/pull/47 is merged, we can remove this block
    from rospkg import RosPack
    if RosPack().get_manifest('urdfdom_py').version == '0.4.0':
        for l in robot.links:
            if l.visual:
                l.add_aggregate('visual', l.visual)
            if l.collision:
                l.add_aggregate('collision', l.collision)

    robot.add_aggregate('gazebo', etree.fromstring('<gazebo>'
                                                   '<plugin filename="libgazebo_ros_control.so" name="gazebo_ros_control">'
                                                   '<robotNamespace>/</robotNamespace>'
                                                   '<robotSimType>gazebo_ros_control/DefaultRobotHWSim</robotSimType>'
                                                   '<legacyModeNS>true</legacyModeNS>'
                                                   '</plugin>'
                                                   '<plugin name="ground_truth" filename="libgazebo_ros_p3d.so">'
                                                   '<frameName>world</frameName>'
                                                   '<bodyName>base_link</bodyName>'
                                                   '<topicName>base_link_ground_truth</topicName>'
                                                   '<updateRate>30.0</updateRate>'
                                                   '</plugin>'
                                                   '</gazebo>'))


# write urdf file
def write_urdf_file(name, robot):
    f = open('urdf/{}.urdf'.format(name), 'w')
    print("writing urdf file to %s" % f.name)
    f.write(
        '<?xml version="1.0" ?>\n'
        '<!--\n'
        '  This file is automatically generated by ' + (' '.join(sys.argv)).replace('--', '\-\-') + '\n'
        '  and distributed under the TERMS OF USE FOR GUNDAM RESEARCH OPEN SIMULATOR Attribution-NonCommercial-ShareAlike\n'
        '  Copyright: SOTSU, SUNRISE / GUNDAM GLOBAL CHALLENGE\n'
        '-->\n')
    f.write(robot.to_xml_string().split("\n", 1)[1])  # skip <?xml version="1.0" ?>
    f.close()


# write ros_control configuration file
def write_control_file(joints_dict):
    # write control file
    f = open('../gundam_rx78_control/config/gundam_rx78_control.yaml', 'w')
    print("Writing ros_control config file to %s" % f.name)
    f.write('# Publish all joint states -----------------------------------\n'
            'joint_state_controller:\n'
            '  type: joint_state_controller/JointStateController\n'
            '  publish_rate: 50\n'
            '\n')
    f.write('# Position Controllers ---------------------------------------\n')
    for i, j in joints_dict.items():
        if ('name' in j) and (not ('mimic' in j)):
            f.write('%s_position:\n' % j['name'])
            f.write('  type: %s_controllers/JointPositionController\n' %
                    args.controller_type)
            f.write('  joint: %s\n' % j['name'])
            if 'pid' in j:
                f.write('  pid: {p: %f, i: %f, d: %f}\n' % (j['pid']['p'], j['pid']['i'], j['pid']['d']))
            else:
                f.write('  pid: {p: %f, i: %f, d: %f}\n' % (default_pid['p'], default_pid['i'], default_pid['d']))
            # for mimic joints
            if not args.no_mimic:
                has_mimic_joints = False
                for ii, jj in joints_dict.items():
                    if 'mimic' in jj and jj['mimic'] == j['name']:
                        if not has_mimic_joints:
                            f.write('  mimic_joints:\n')
                            has_mimic_joints = True
                        f.write('    %s_position:\n' % jj['name'])
                        f.write('      type: %s_controllers/JointPositionController\n' % args.controller_type)
                        f.write('      joint: %s\n' % jj['name'])
                        if 'pid' in jj:
                            f.write('      pid: {p: %f, i: %f, d: %f}\n' % (jj['pid']['p'], jj['pid']['i'], jj['pid']['d']))
                        elif 'pid' in j:
                            f.write('      pid: {p: %f, i: %f, d: %f}\n' % (j['pid']['p'], j['pid']['i'], j['pid']['d']))
                        else:
                            f.write('      pid: {p: %f, i: %f, d: %f}\n' % (default_pid['p'], default_pid['i'], default_pid['d']))

    f.write('\n')
    f.write('# Joint Trajectory Controllers ---------------------------------------\n')
    f.write('fullbody_controller:\n')
    f.write('#  type: %s_controllers/JointTrajectoryController\n' % args.controller_type)
    f.write('  type: gundam_rx78_control/JointTrajectoryController\n')
    f.write('  joints:\n')
    for i, j in joints_dict.items():
        if ('name' in j) and not ('mimic' in j):
            f.write('    - %s\n' % j['name'])
    # for mimic joints
    if not args.no_mimic:
        f.write('  mimic_joints:\n')
        for i, j in joints_dict.items():
            if 'mimic' in j:
                f.write('    - %s # %s\n' % (j['name'], j['mimic']))
    # gains
    f.write('  gains:\n')
    for i, j in joints_dict.items():
        if ('name' in j) and not ('mimic' in j):
            if 'pid' in j:
                f.write('    %s : {p: %f, i: %f, d: %f}\n' % (j['name'], j['pid']['p'], j['pid']['i'], j['pid']['d']))
            else:
                f.write('    %s : {p: %f, i: %f, d: %f}\n' % (j['name'], default_pid['p'], default_pid['i'], default_pid['d']))
    # for mimic joints
    if not args.no_mimic:
        for i, j in joints_dict.items():
            if 'mimic' in j:
                if 'pid' in j:
                    f.write('    %s : {p: %f, i: %f, d: %f}' % (j['name'], j['pid']['p'], j['pid']['i'], j['pid']['d']))
                elif [item for key, item in joints_dict.items() if (item['name'] == j['mimic']) and ('pid' in item)] != None:
                    jj = [item for key, item in joints_dict.items() if (item['name'] == j['mimic']) and ('pid' in item)][0]
                    f.write('    %s_joint : {p: %f, i: %f, d: %f}' % (j['name'], jj['pid']['p'], jj['pid']['i'], jj['pid']['d']))
                else:
                    f.write('    %s_joint : {p: %f, i: %f, d: %f}' % (j['name'], default_pid['p'], default_pid['i'], default_pid['d']))
                f.write('    # mimic joint of %s_joint\n' % (j['mimic']))
    f.write('  constraints:\n')
    f.write('    goal_time: 0.6\n')
    f.write('    stopped_velocity_tolerance: 0.05\n')
    f.write('    # joint: {trajectory: 0.2, goal: 0.2}\n')
    f.write('  stop_trajectory_duration: 0.5\n')
    f.write('  state_publish_rate:  125\n')
    f.write('  action_monitor_rate: 10\n')
    f.write('  allow_partial_joints_goal: true\n')


global robot, args
if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('input_file', help='input collada file name')
    parser.add_argument('--controller_type', choices=[
                        'position', 'velocity', 'effort'], default='effort', help='set controller type')
    parser.add_argument(
        '--no_mimic', action='store_true', help='disable mimic joint')
    parser.add_argument(
        '--pin', action='store_true', help='pin the robot to the world')
    parser.add_argument(
        '--write_mesh', action='store_true', help='write mech files')
    args = parser.parse_args()

    # load collada file
    mesh_ = Collada(args.input_file)
    if mesh_.xmlnode.getroot().attrib['version'] != '1.4.1':
        print('This program only support COLLADA 1.4.1, but the input file is %s' %
              mesh_.xmlnode.getroot().attrib['version'])
        sys.exit(1)

    # remove unused geometries / materials / effects / animations
    simplify_collada(mesh_)

    # merge nodes into one node if no joints exist between them.
    # add additional nodes if multiple joints exist for one childnode
    # add root link
    mergenode_collada(mesh_, joints_, root_offset)
    joints_dict = dict(joints_)

    # apply scale
    scale_collada(mesh_, scale_)

    # extract robot name
    name_ = os.path.splitext(os.path.basename(mesh_.filename))[0]

    # create robot instance
    robot_ = Robot(name=name_)
    # DEBUG: create pinned model
    if args.pin:
        robot_.add_link(Link(name='world'))
        robot_.add_joint(
            Joint(name='world_to_base', parent='world', child='base_link', joint_type='fixed'))
    # robot_.add_link(Link(name='base_link'))
    print("loaded collada file {}".format(name_))
    link_dict = dict()
    retrive_node(mesh_.scene.nodes[0].children, joints_dict, link_dict)  # hack for base_link

    # update transmission joints to human readable ones
    # update_joint_name(robot_, joints_dict)

    # add gazebo information
    add_gazebo_nodes(robot_, link_dict)

    print('all weight is %f' % all_weight_)

    # write urdf file
    write_urdf_file(name_, robot_)

    # write control file
    write_control_file(joints_dict)
