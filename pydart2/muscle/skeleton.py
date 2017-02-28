# Author(s): Sehoon Ha <sehoon.ha@gmail.com>
import numpy as np
from pydart2.skeleton import Skeleton


class MusculoSkeleton(Skeleton):
    """
    world = pydart2.world.World(0.001)
    world.add_skeleton("data/skeleton_file", MusculoSkeleton)
    """

    def __init__(self, _world, _filename=None,
                 _id=None, _friction=None, ):
        Skeleton.__init__(self, _world, _filename, _id, _friction)
        self.muscles = list()

    def num_muscles(self, ):
        return len(self.muscles)

    def add_muscle(self, mtu, route):
        route.initialize_points(self)
        self.muscles.append((mtu, route))

    def update(self, ):
        for mtu in self.tendon_units:
            mtu.update()

    def musle_torques(self, ):
        pass

    def render_with_ri(self, ri):
        r = np.array([1.0, 0.0, 0.0])
        b = np.array([0.0, 0.0, 1.0])
        for mtu, route in self.muscles:
            w = mtu.a
            color = (1 - w) * b + w * r
            ri.set_color(*color)
            route.render_with_ri(ri)

    def __repr__(self):
        args = (self.id, self.name, self.num_muscles())
        return '[MusculoSkeleton(%d): %s # Muscles = %d]' % args
