if __name__ == '__main__':
    import sys
    import pydart2 as pydart
    print('Hello, PyDART!')

    if len(sys.argv) != 2:
        print("Usage: inspect_skeleton.py [path_to_skeleton]")
        exit(0)

    skel_path = sys.argv[1]
    print("skeleton path = %s" % skel_path)

    pydart.init()
    print("pydart init OK")

    world = pydart.World(1.0 / 1000.0)
    print("World init OK")

    skel = world.add_skeleton(skel_path)
    print("Skeleton add OK")

    print('----------------------------------------')
    print('[Basic information]')
    print('mass = %.6f' % skel.m)
    print('# DoFs = %s' % skel.ndofs)

    print('[DegreeOfFreedom]')
    for i, dof in enumerate(skel.dofs):
        print("DoF %d: %s" % (i, dof))

    print('[Position]')
    print('positions = %s' % str(skel.q))
    print('velocities = %s' % str(skel.dq))
    print('states = %s' % str(skel.x))

    print('[Lagrangian]')
    print('mass matrix = %s' % str(skel.M))
    print('coriolis_and_gravity_forces = %s' % str(skel.c))
    print('constraint_forces = %s' % str(skel.constraint_forces()))
    print('----------------------------------------')

    pydart.gui.viewer.launch(world)
