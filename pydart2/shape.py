from __future__ import absolute_import
from builtins import object
# Copyright (c) 2015, Disney Research
# All rights reserved.
#
# Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
# Disney Research Robotics Group
from . import pydart2_api as papi


class Shape(object):
    """
    """
    def __init__(self, _shapenode):
        """
        """
        self.shapenode = _shapenode
        self.bodynode = _shapenode.bodynode
        self.skeleton = _shapenode.bodynode.skeleton
        self.id = _shapenode.id

    @property
    def skel(self):
        return self.skeleton

    @property
    def wid(self):
        return self.skel.world.id

    @property
    def skid(self):
        return self.skel.id

    @property
    def bid(self):
        return self.bodynode.id

    def build(self):
        pass

    def volume(self, ):
        return papi.shape__getVolume(self.wid, self.skid, self.bid, self.id)

    def shape_type(self, ):
        names = ["BOX", "ELLIPSOID", "CYLINDER", "PLANE",
                 "MESH", "SOFT_MESH", "LINE_SEGMENT", ]
        return names.index(self.shape_type_name())

    def shape_type_name(self, ):
        ret = papi.shape__getShapeType(self.wid, self.skid, self.bid, self.id)
        # if ret == "SPHERE":
        #     ret = "ELLIPSOID"  # Backward compatibility
        return ret

    def render(self, ):
        papi.shape__render(self.wid, self.skid, self.bid, self.id)

    def bounding_box(self, ):
        _min = papi.shape__getBoundingBoxMin(self.wid,
                                             self.skid,
                                             self.bid,
                                             self.id)
        _max = papi.shape__getBoundingBoxMax(self.wid,
                                             self.skid,
                                             self.bid,
                                             self.id)
        return (_min, _max)

    def __repr__(self):
        return '[Shape(%d:%d)]' % (self.bid, self.id)
