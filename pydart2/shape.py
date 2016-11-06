from __future__ import absolute_import
# from builtins import object
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
        type_ = papi.shape__getShapeType(self.wid, self.skid,
                                         self.bid, self.id)
        type_ = 1 if type_ == -1 else type_  # Assume to be ELLIPSOID
        return type_

    def shape_type_name(self, ):
        type_ = self.shape_type()
        names = ["BOX", "ELLIPSOID", "CYLINDER", "PLANE",
                 "MESH", "SOFT_MESH", "LINE_SEGMENT"]
        return names[type_]

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


class BoxShape(Shape):
    def __init__(self, _shapenode):
        Shape.__init__(self, _shapenode)

    def size(self, ):
        return papi.box_shape__getSize(self.wid, self.skid, self.bid, self.id)

    def set_size(self, _size):
        return papi.box_shape__setSize(self.wid, self.skid, self.bid, self.id,
                                       _size)

    def __repr__(self):
        return '[BoxShape(%d:%d)]' % (self.bid, self.id)


class EllipsoidShape(Shape):
    def __init__(self, _shapenode):
        Shape.__init__(self, _shapenode)

    def size(self, ):
        return papi.ellipsoid_shape__getSize(self.wid, self.skid,
                                             self.bid, self.id)

    def set_size(self, _size):
        return papi.ellipsoid_shape__setSize(self.wid, self.skid,
                                             self.bid, self.id,
                                             _size)

    def __repr__(self):
        return '[EllipsoidShape(%d:%d)]' % (self.bid, self.id)


class CylinderShape(Shape):
    def __init__(self, _shapenode):
        Shape.__init__(self, _shapenode)

    def __repr__(self):
        return '[CylinderShape(%d:%d)]' % (self.bid, self.id)


class PlaneShape(Shape):
    def __init__(self, _shapenode):
        Shape.__init__(self, _shapenode)

    def __repr__(self):
        return '[PlaneShape(%d:%d)]' % (self.bid, self.id)


class MeshShape(Shape):
    def __init__(self, _shapenode):
        Shape.__init__(self, _shapenode)

    def scale(self, ):
        return papi.mesh_shape__getScale(self.wid, self.skid,
                                         self.bid, self.id)

    def path(self, ):
        return papi.mesh_shape__getMeshPath(self.wid, self.skid,
                                            self.bid, self.id)

    def __repr__(self):
        return '[MeshShape(%d:%d)]' % (self.bid, self.id)


class SoftMeshShape(Shape):
    def __init__(self, _shapenode):
        Shape.__init__(self, _shapenode)

    def __repr__(self):
        return '[SoftMeshShape(%d:%d)]' % (self.bid, self.id)


class LineSegmentShape(Shape):
    def __init__(self, _shapenode):
        Shape.__init__(self, _shapenode)

    def __repr__(self):
        return '[LineSegmentShape(%d:%d)]' % (self.bid, self.id)


def create_shape(shapenode):
    bodynode = shapenode.bodynode
    skeleton = bodynode.skeleton
    world = skeleton.world

    wid, skid, bid, id = world.id, skeleton.id, bodynode.id, shapenode.id

    type_ = papi.shape__getShapeType(wid, skid, bid, id)
    type_ = 1 if type_ == -1 else type_  # Assume to be ELLIPSOID
    shape_classes = [BoxShape, EllipsoidShape, CylinderShape, PlaneShape,
                     MeshShape, SoftMeshShape, LineSegmentShape]
    if type_ < len(shape_classes):
        cls = shape_classes[type_]
        return cls(shapenode)
    else:
        print("Invalid type: %d" % type_)
        return None
