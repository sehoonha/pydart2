
if __name__ == '__main__':
    import sys
    import pydart2 as pydart

    if len(sys.argv) != 2:
        print("Usage: inspect_skeleton.py [*.urdf/*.sdf]")
        exit(0)

    skel_path = sys.argv[1]
    print("skeleton path = %s" % skel_path)

    pydart.init()
    print("pydart init OK")

    world = pydart.World(0.0002)
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
    print('root_bodynode[0] = ' + str(skel.root_bodynode(index=0)))
    for body in skel.bodynodes:
        print("\t" + str(body))
        print("\t\tmass = %.4fKg" % body.m)
        print("\t\tparent = " + str(body.parent_bodynode))
        print("\t\tchilds = " + str(body.child_bodynodes))
        print("\t\tCOM = " + str(body.C))
        print("\t\t# dependent dofs = %d" % len(body.dependent_dofs))
        print("\t\t# shapenodes = %s" % str(body.shapenodes))
        print("\t\t# markers = %d" % len(body.markers))
        # print("J = %s" % str(body.J))

    print('[DegreeOfFreedom]')
    for dof in skel.dofs:
        print("\t" + str(dof) + " belongs to " + str(dof.joint))
        # print("\t\tindex in skeleton = " + str(dof.index_in_skeleton()))
        # print("\t\tposition = " + str(dof.position()))

    print('[Joint]')
    for joint in skel.joints:
        print("\t" + str(joint))
        print("\t\tparent = " + str(joint.parent_bodynode))
        print("\t\tchild = " + str(joint.child_bodynode))
        print("\t\tdofs = " + str(joint.dofs))

    print('[Markers]')
    for marker in skel.markers:
        print("\t" + str(marker) + " attached to " + str(marker.bodynode))
        print("\t\t" + str(marker.world_position()))

    print('[Position]')
    print('\tpositions = %s' % str(skel.q))
    print('\tvelocities = %s' % str(skel.dq))
    print('\tstates = %s' % str(skel.x))

    print('[Limits]')
    print('\tposition_lower_limits = %s' % str(skel.q_lower))
    print('\tposition_upper_limits = %s' % str(skel.q_upper))
    print('\tforce_lower_limits = %s' % str(skel.tau_lower))
    print('\tforce_upper_limits = %s' % str(skel.tau_upper))

    print('[Lagrangian]')
    print('\tmass matrix = %s' % str(skel.M))
    print('\tcoriolis_and_gravity_forces = %s' % str(skel.c))
    print('\tconstraint_forces = %s' % str(skel.constraint_forces()))
    print('----------------------------------------')
