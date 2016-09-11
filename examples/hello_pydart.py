from __future__ import division
from __future__ import print_function
from builtins import str
from past.utils import old_div
import pydart2 as pydart

if __name__ == '__main__':
    print('Hello, PyDART!')

    pydart.init()
    print('pydart initialization OK')

    world = pydart.World(old_div(1.0, 2000.0), './data/skel/cubes.skel')
    print('pydart create_world OK')

    while world.t < 2.0:
        if world.nframes % 100 == 0:
            skel = world.skeletons[-1]
            print("%.4fs: The last cube COM = %s" % (world.t, str(skel.C)))
        world.step()
