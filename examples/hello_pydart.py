if __name__ == '__main__':
    import pydart2 as pydart
    print('Hello, PyDART!')

    pydart.init()
    print('pydart initialization OK')

    import os
    data_dir = os.getcwd() + "/data/"
    print('data_dir = ' + data_dir)

    world = pydart.create_world(1.0 / 2000.0, data_dir + 'skel/cubes.skel')
    print('pydart create_world OK')

    for skel in world.skeletons:
        print skel.name

    while world.t < 2.0:
        # if True:
        if world.nframes % 100 == 0:
            q = world.skeletons[2].q
            print("%.4fs: The third cube pos = %s" % (world.t, str(q)))
        world.step()
        # print(str(world))

    # pydart.gui.viewer.launch(world)
