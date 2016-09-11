from __future__ import division
from __future__ import print_function
from past.utils import old_div
# Copyright (c) 2015, Disney Research
# All rights reserved.
#
# Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
# Disney Research Robotics Group
import pydart2 as pydart
if __name__ == '__main__':
    pydart.init()
    print('pydart initialization OK')

    world = pydart.World(old_div(1.0, 2000.0), './data/skel/softBodies.skel')
    print('pydart create_world OK')

    pydart.gui.viewer.launch(world)
