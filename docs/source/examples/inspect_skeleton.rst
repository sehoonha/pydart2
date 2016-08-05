Inspect Skeleton: Navigating Bodynodes, Joints, and Dofs 
=========================================================
This example demonstrates how to navigate the components of the given
skeleton including body nodes, joints, dofs, shapes, markers, and so on.

.. code-block:: bash

   python inspect_skeleton.py data/sdf/atlas/atlas_v3_no_head.sdf

Result
^^^^^^^^^^^^

.. code-block:: bash

    ----------------------------------------
    [Basic information]
            mass = 146.554000
            # DoFs = 33
    [BodyNode]
    root_bodynode[0] = [BodyNode(0): pelvis]
            [BodyNode(0): pelvis]
                    mass = 17.8820Kg
                    parent = None
                    childs = [[BodyNode(1): ltorso], [BodyNode(9): l_uglut], [BodyNode(21): r_uglut]]
                    COM = [ 0.0111  0.      0.0271]
                    # dependent dofs = 6
                    # shapenodes = [[ShapeNode(0:0)], [ShapeNode(0:1)]]
                    # markers = 0
            [BodyNode(1): ltorso]
                    mass = 2.4090Kg
                    parent = [BodyNode(0): pelvis]
                    childs = [[BodyNode(2): mtorso]]
                    COM = [ -2.37984000e-02  -3.15366000e-06   7.46835000e-02]
                    # dependent dofs = 7
                    # shapenodes = [[ShapeNode(1:0)], [ShapeNode(1:1)]]
                    # markers = 0
            [BodyNode(2): mtorso]
                    mass = 0.6900Kg
                    parent = [BodyNode(1): ltorso]
                    childs = [[BodyNode(3): utorso]]
                    COM = [-0.02066266 -0.0131245   0.1925674 ]
                    # dependent dofs = 8
                    # shapenodes = [[ShapeNode(2:0)], [ShapeNode(2:1)]]
                    # markers = 0
    ...                 

Code
^^^^^^^^^^^^

.. code-block:: python
   :linenos:

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
            print("J = %s" % str(body.J))

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
