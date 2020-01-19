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

from collada import *
from urdf_parser_py.urdf import *
from tf.transformations import *
import copy
import os
import sys
import math
import numpy
import argparse


def scale_node(node, scale):
    transforms = node.transforms
    node.transforms = []
    for transform in transforms:
        transform.matrix[:3, 3] *= scale
        node.transforms.append(scene.MatrixTransform(transform.matrix.reshape(16, 1)))
    node.matrix = numpy.identity(4, dtype=numpy.float32)
    for t in node.transforms:
        node.matrix = numpy.dot(node.matrix, t.matrix)

    for child in node.children:
        if isinstance(child, scene.Node):
            scale_node(child, scale)


def scale_geometry(geometry, scale):
    position_ids = set()
    for primitive in geometry.primitives:
        for _input in primitive.getInputList().getList():
            if _input[1] == "VERTEX":
                position_ids.add(_input[2][1:])  # remove '#'
    for position_id in position_ids:
        geometry.sourceById[position_id].data *= scale


def scale_collada(mesh_, scale):
    scale_node(mesh_.scene.nodes[0], scale)
    for geometry in mesh_.geometries:
        scale_geometry(geometry, scale)

    mesh_.assetInfo.unitname = 'meter'
    mesh_.assetInfo.unitmeter = 1.0
    mesh_.save()
