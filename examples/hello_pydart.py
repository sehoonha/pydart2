if __name__ == '__main__':
    import pydart2 as pydart
    print('Hello, PyDART!')

    pydart.init()
    print('pydart initialization OK')

    world = pydart.World(1.0 / 2000.0, './data/skel/cubes.skel')
    print('pydart create_world OK')

    for idx, skel in enumerate(world.skeletons):
        print("%dth Skeleton: %s" % (idx, skel.name))

    # while world.t < 2.0:
    #     # if True:
    #     if world.nframes % 100 == 0:
    #         q = world.skeletons[2].q
    #         print("%.4fs: The third cube pos = %s" % (world.t, str(q)))
    #     world.step()
    #     # print(str(world))

    pydart.gui.viewer.launch(world)
