# Author(s): Sehoon Ha <sehoon.ha@gmail.com>


class MusculoSkeleton(object):
    """
    world = pydart2.world.World(0.001)
    world.add_skeleton("data/skeleton_file", MusculoSkeleton)
    """
    def __init__(self, ):
        self.tendon_units = list()

    def update(self, ):
        for mtu in self.tendon_units:
            mtu.update()

    def musle_torques(self, ):
        pass
