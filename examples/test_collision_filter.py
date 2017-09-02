# Copyright (c) 2015, Disney Research
# All rights reserved.
#
# Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
# Disney Research Robotics Group

import pydart2 as pydart
if __name__ == '__main__':
    pydart.init()
    print('pydart initialization OK')

    world = pydart.World(0.0002, './data/skel/shapes.skel')
    print('pydart create_world OK')

    collision_filter = world.create_collision_filter()
    print('pydart collision_filter OK')

    body0 = world.skeletons[0].bodynodes[0]
    for index in [2, 3, 4, 6]:
        body1 = world.skeletons[index].bodynodes[0]
        print("disable collision between %s and %s" % (body0, body1))
        collision_filter.add_to_black_list(body0, body1)

    for index in [4]:
        body1 = world.skeletons[index].bodynodes[0]
        print("(re)enable collision between %s and %s" % (body0, body1))
        collision_filter.remove_from_black_list(body0, body1)
    # collision_filter.remove_all_from_black_list()

    pydart.gui.viewer.launch(world)
