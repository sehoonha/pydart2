if __name__ == '__main__':
    import pydart2 as pydart
    from pydart2.ik import IK
    import numpy as np
    import time
    pydart.init()
    world = pydart.world.World(0.001)
    world.set_collision_detector(2)
    robot = world.add_skeleton("./data/vsk/400.vsk")

    ik = IK()
    ik.set_skeleton(robot)
    ik.add_body_and_offset(robot.bodynodes[3], np.array((0.0, 0.0, 1.0)))
    ik.add_body_and_offset(robot.bodynodes[4], np.array((0.0, 0.5, 0.0)))

    X = np.random.rand(2, 3)
    tic = time.time()
    ik.set_targets(X)
    toc = time.time()
    print("elapsed = %.6fs" % (toc - tic))

    for i in range(ik.num()):
        print("entry", i)
        print("\tlhs", ik.body_name(i), ik.offset(i))
        print("\trhs", ik.target(i))
    # ik.add_body_and_offset(robo)
    # ik.set_target_position(0, np.random.rand(3))
