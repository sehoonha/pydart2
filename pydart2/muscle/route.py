# Author(s): Sehoon Ha <sehoon.ha@gmail.com>
#          : Seungmoon Song <ssm0445@gmail.com>
import numpy as np
from pydart2.utils.misc import S


class AttachmentPoint(object):
    """
    """
    def __init__(self, bodyname, offset):
        self.bodyname = bodyname
        self.skeleton = None
        self.body = None
        self.offset = np.array(offset)

    def is_initialized(self, ):
        return (self.body is not None)

    def initialize(self, skeleton):
        self.skeleton = skeleton
        self.body = skeleton.body(self.bodyname)

    def to_world(self, ):
        return self.body.to_world(self.offset)

    def __str__(self, ):
        return "(%s, %s)" % (self.bodyname, S(self.offset, 3))


class Route(object):
    """
    route = Route([("Upper", [0.0, 0.2, 0.0]), ("Lower", [0.0, 0.2, 0.0])])
    """
    def __init__(self, points=None):
        if points is None:
            self.points = []
        else:
            self.points = [AttachmentPoint(name, offset)
                           for name, offset in points]

    def num_points(self, ):
        return len(self.points)

    def __len__(self, ):
        return self.num_points()

    def add_point(self, bodyname, offset):
        pt = AttachmentPoint(bodyname=bodyname,
                             offset=offset)
        self.points.append(pt)

    def initialize_points(self, skeleton):
        for pt in self.points:
            pt.initialize(skeleton)

    def render_with_ri(self, ri, ):
        if self.num_points() < 2:
            return
        ri.set_line_width(3)
        world_points = [pt.to_world() for pt in self.points]
        ri.render_lines(world_points)

    def length(self, ):
        world_points = [pt.to_world() for pt in self.points]
        length = 0.0
        pt0 = world_points[0]
        for pt1 in world_points[1:]:
            length += np.linalg.norm(pt0 - pt1)
            pt0 = pt1
        return length

    def __repr__(self, ):
        tokens = [str(pt) for pt in self.points]
        return "[%s: length = %.4f]" % (", ".join(tokens), self.length())
