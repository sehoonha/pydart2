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

    while world.t < 2.0:
        # if world.nframes % 100 == 0:
        #     print("%.4fs: The third cube pos = " % (world.t))
        world.step()
        print(str(world))
