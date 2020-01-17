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


def search_used_geom_mat(nodes, used_geometries_id, used_materials_id):
    for node in nodes:
        if isinstance(node, scene.GeometryNode):
            used_geometries_id.add(node.geometry.id)

            # remove empty triangles
            active_primitives_in_geometry = []
            for primitive in node.geometry.primitives:
                if isinstance(primitive, triangleset.TriangleSet):
                    if primitive.ntriangles != 0:
                        active_primitives_in_geometry.append(primitive)
                else:
                    print("{} is not supported".format(type(primitive)))
            node.geometry.primitives = active_primitives_in_geometry

            # remove unsed material refs
            used_materials_in_geometry = set()
            for primitive in node.geometry.primitives:
                used_materials_in_geometry.add(primitive.material)
            active_materials_in_geometry = []
            for material in node.materials:
                if material.symbol in used_materials_in_geometry:
                    active_materials_in_geometry.append(material)
            node.materials = active_materials_in_geometry

            for material in node.materials:
                used_materials_id.add(material.target.id)

        if isinstance(node, scene.Node):
            search_used_geom_mat(node.children, used_geometries_id, used_materials_id)


def simplify_collada(mesh_):
    used_geometries_id = set()
    used_materials_id = set()
    search_used_geom_mat(mesh_.scene.nodes, used_geometries_id, used_materials_id)

    # remove unused geometries
    active_geometries = []
    for geometry in mesh_.geometries:
        if geometry.id in used_geometries_id:
            active_geometries.append(geometry)
    mesh_.geometries = active_geometries

    # remove unused materials
    active_materials = []
    for material in mesh_.materials:
        if material.id in used_materials_id:
            active_materials.append(material)
    mesh_.materials = active_materials

    # remove unused effects
    used_effects_id = set()
    for material in mesh_.materials:
        used_effects_id.add(material.effect.id)
    active_effects = []
    for effect in mesh_.effects:
        if effect.id in used_effects_id:
            active_effects.append(effect)
    mesh_.effects = active_effects

    # remove animations
    mesh_.animations = []
    # Because library_animations is not modified by Collada.save() function, library_animations needs to be removed manually.
    node = mesh_.xmlnode.find(tag("library_animations"))
    if node is not None:
        mesh_.xmlnode.getroot().remove(node)

    mesh_.save()
