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

    world.g = [0.0, 0.0, -9.8]
    print("gravity = %s" % str(world.g))

    skel = world.add_skeleton(skel_path)
    print("Skeleton add OK")

    print('----------------------------------------')
    print('[Basic information]')
    print('\tmass = %.6f' % skel.m)
    print('\t# DoFs = %s' % skel.ndofs)

    print('[BodyNode]')
    for body in skel.bodynodes:
        print("\t" + str(body))

    print('[DegreeOfFreedom]')
    for dof in skel.dofs:
        print("\t" + str(dof))

    print('[Position]')
    print('\tpositions = %s' % str(skel.q))
    print('\tvelocities = %s' % str(skel.dq))
    print('\tstates = %s' % str(skel.x))

    print('[Lagrangian]')
    print('\tmass matrix = %s' % str(skel.M))
    print('\tcoriolis_and_gravity_forces = %s' % str(skel.c))
    print('\tconstraint_forces = %s' % str(skel.constraint_forces()))
    print('----------------------------------------')

    pydart.gui.viewer.launch(world)
