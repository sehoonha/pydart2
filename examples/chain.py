# Copyright (c) 2015, Disney Research
# All rights reserved.
#
# Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
# Disney Research Robotics Group
import numpy as np


class DampingController(object):
    """ Add damping force to the skeleton """
    def __init__(self, skel):
        self.skel = skel

    def compute(self):
        damping = -0.01 * self.skel.dq
        damping[1::3] *= 0.1
        return damping


if __name__ == '__main__':
    import pydart2 as pydart

    pydart.init(verbose=True)
    print('pydart initialization OK')

    world = pydart.World(0.0002, './data/skel/chain.skel')
    print('pydart create_world OK')

    skel = world.skeletons[0]
    skel.q = (np.random.rand(skel.ndofs) - 0.5)
    print('init pose = %s' % skel.q)
    skel.controller = DampingController(skel)

    pydart.gui.viewer.launch(world)

    # # Or, you can manually create the window...
    # win = pydart.gui.viewer.PydartWindow(world)
    # win.camera_event(1)
    # win.run_application()
