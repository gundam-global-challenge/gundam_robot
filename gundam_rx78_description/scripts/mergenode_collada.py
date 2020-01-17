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


def mergenode(parentnode, childnode):
    if isinstance(parentnode, scene.Node) and isinstance(childnode, scene.Node):
        # merge childnode into parentnode
        parentnode.children.remove(childnode)
        child_mat = childnode.matrix
        for child in childnode.children:
            if isinstance(child, scene.Node):
                child.transforms = copy.deepcopy(childnode.transforms) + child.transforms
                for t in reversed(childnode.transforms):
                    child.matrix = numpy.dot(t.matrix, child.matrix)
                parentnode.children.append(child)
            elif isinstance(child, scene.GeometryNode):
                position_ids = set()
                normal_ids = set()
                for primitive in child.geometry.primitives:
                    for _input in primitive.getInputList().getList():
                        if _input[1] == "VERTEX":
                            position_ids.add(_input[2][1:])  # remove '#'
                        elif _input[1] == "NORMAL":
                            normal_ids.add(_input[2][1:])  # remove '#'
                for position_id in position_ids:
                    child.geometry.sourceById[position_id].data = ((child_mat[:3, :3].astype("float64").dot(child.geometry.sourceById[position_id].data.transpose())).transpose() + child_mat[:3, 3]).copy()
                for normal_id in normal_ids:
                    child.geometry.sourceById[normal_id].data = (child_mat[:3, :3].astype("float64").dot(child.geometry.sourceById[normal_id].data.transpose())).transpose().copy()
                for primitive in child.geometry.primitives:
                    primitive._vertex = primitive.sources['VERTEX'][0][4].data
                    primitive._normal = primitive.sources['NORMAL'][0][4].data
                parentnode.children.append(child)


def get_merged(node, joints_dict):
    if isinstance(node, scene.Node):
        # merge nodes below this node unless joint exists
        for child in node.children[:]:
            if isinstance(child, scene.Node):
                if child.id in joints_dict:
                    get_merged(child, joints_dict)
                else:
                    mergenode(node, get_merged(child, joints_dict))
    return node


def find_parent_node(node, name):
    for child in node.children:
        if isinstance(child, scene.Node):
            if child.id == name:
                return node, child
            else:
                ret = find_parent_node(child, name)
                if ret:
                    return ret
    return False


def mergenode_collada(mesh_, joints_, root_offset):
    # apply root_offset
    node = mesh_.scene.nodes[0]
    node.transforms = [scene.MatrixTransform(root_offset.copy().reshape(16, 1))] + node.transforms
    node.matrix = numpy.dot(root_offset, node.matrix)
    newnode = scene.Node("offset_link", children=[node])
    newnode.xmlnode.set("sid", "offset_link")
    newnode.transforms = [scene.MatrixTransform(numpy.identity(4, dtype=numpy.float32).reshape(16, 1))]
    mesh_.scene.nodes = [newnode]

    # merge nodes into one node if no joints exist between them.
    joints_dict = dict(joints_)
    get_merged(mesh_.scene.nodes[0], joints_dict)
    mesh_.save()

    # add additional nodes if multiple joints exist for one childnode
    joints_new = copy.deepcopy(joints_)
    childlink_count = dict()
    for joint in joints_:
        if joint[0] in childlink_count.keys():
            childlink_count[joint[0]] += 1
        else:
            childlink_count[joint[0]] = 1

    for childlinkid, count in childlink_count.items():
        if count > 1:
            parentnode, childnode = find_parent_node(mesh_.scenes[0].nodes[0], childlinkid)
            for i in range(count - 1):
                parentnode.children.remove(childnode)
                newnode = scene.Node(childnode.id + "_addition_null" + str(i), children=[childnode])
                newnode.xmlnode.set("sid", childnode.id + "_addition_null" + str(i))
                newnode.transforms = childnode.transforms
                newnode.matrix = childnode.matrix
                childnode.transforms = []
                childnode.matrix = numpy.identity(4, dtype=numpy.float32)
                parentnode.children.append(newnode)
                parentnode = newnode
                j = [s for s in joints_new if s[0] == childlinkid][0]
                j[0] += "_addition_null" + str(i)

    # add root link
    if len(mesh_.scene.nodes[0].children) == 1:
        mesh_.scene.nodes[0].id = "visual0"
        mesh_.scene.nodes[0].xmlnode.set("sid", "visual0")
    else:
        node = mesh_.scene.nodes[0]
        newnode = scene.Node("visual0", children=[node])
        newnode.xmlnode.set("sid", "visual0")
        mesh_.scene.nodes = [newnode]
    mesh_.scene.nodes[0].children[0].id = "base_link"
    mesh_.scene.nodes[0].children[0].xmlnode.set("sid", "base_link")
    mesh_.scene.nodes[0].children[0].transforms = [scene.MatrixTransform(numpy.identity(4, dtype=numpy.float32).reshape(16, 1))]
    mesh_.scene.nodes[0].children[0].matrix = numpy.identity(4, dtype=numpy.float32)

    mesh_.save()
    joints_[:] = joints_new
