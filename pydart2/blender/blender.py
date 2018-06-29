import numpy as np
import bpy
import pydart2.blender.blender_tools as bt


def load_skeleton(filename, collision_detector=None):
    print("load_skeleton: [%s]" % filename)
    world, skel = create_world_and_skel(filename, collision_detector)
    meshdata = create_meshes(skel)
    arma = create_bones(skel)
    for obj, body in meshdata:
        print("attach_to_bone...: %s" % (str(obj)))
        bone = arma.data.bones[body.name]
        bt.attach_to_bone2(obj, arma, bone)
        print("attach_to_bone OK: %s %s" % (str(obj), str(bone)))


def create_world_and_skel(filename, collision_detector=None):
    print("create_world_and_skel: [%s]" % filename)
    import pydart2 as pydart
    world = pydart.world.World(0.001)
    if collision_detector is not None:
        print("\tset_collision_detector: %d" % collision_detector)
        world.set_collision_detector(collision_detector)
    print("\tadd_skeleton [%s]" % filename)
    skel = world.add_skeleton(filename)
    print("\tskeleton load OK: [%s]" % str(skel))
    return world, skel


def create_meshes(skel):
    import pydart2.utils.transformations as trans
    import pydart2.shape as ps
    print("create_meshes.....")

    data = list()

    for body in skel.bodynodes:
        print(">>> %s <<<" % str(body))
        T_b = body.world_transform()  # Body transformation
        for shnode in body.shapenodes:
            if not shnode.has_visual_aspect():
                continue
            shape = shnode.shape
            rgba = shnode.visual_aspect_rgba()
            shape_name = str(shape)

            T_s = shnode.relative_transform()  # ShapeNode transformation
            print("\t%s: rgba = %s" % (str(shape), str(rgba)))

            T_t = np.identity(4)  # Temp transformation specific to type
            obj = None
            if isinstance(shape, ps.CapsuleShape):
                r = shape.radius()
                h = shape.height()
                obj = bt.create_capsule(shape_name, r, h, rgba[:3])
                print("\tcreate_capsule", shape_name, r, h, rgba)
                T_t = trans.translation_matrix((0.0, 0.0, -0.5 * h))
            elif isinstance(shape, ps.BoxShape):
                sz = shape.size()
                obj = bt.create_box(shape_name, sz, rgba[:3])
                print("\tcreate_box", shape_name, sz, rgba)
            elif isinstance(shape, ps.MeshShape):
                filename = shape.path()
                filename = filename.replace(".dae", ".stl")
                # obj = import_obj(shape_name, filename, rgba[:3])
                obj = bt.import_stl(shape_name, filename, rgba[:3])
                print("\timport_stl", shape_name, filename, rgba)
            if obj is not None:
                T = T_b.dot(T_s).dot(T_t)
                bt.set_object_transformation(obj, T)
                data.append((obj, body))
    print("create_meshes..... OK")
    return data


def create_bones(skel):
    print("create_bones.....")

    import os.path

    bpy.ops.object.add(
        type='ARMATURE',
        enter_editmode=True,
        location=(0.0, 0.0, 0.0))
    ob = bpy.context.object
    ob.show_x_ray = True
    ob.name = os.path.basename(skel.filename)
    amt = ob.data
    amt.name = ob.name + 'Amt'
    amt.show_axes = True

    bpy.ops.object.mode_set(mode='EDIT')

    for body in skel.bodynodes:
        print(">>> %s <<<" % str(body))
        bone = amt.edit_bones.new(body.name)

        T = body.world_transform()
        R = T[:3, :3]
        pos = T[:3, 3]
        head = pos
        bone.head = head
        print("\thead = %s" % str(head))

        if body.parent_bodynode is not None:
            bone.parent = amt.edit_bones[body.parent_bodynode.name]
            bone.use_connect = False
            print("\tparent bone name = %s" % body.parent_bodynode.name)

        offsets = [sh.offset() for sh in body.shapenodes
                   if sh.has_visual_aspect()]
        if len(offsets) > 0:
            avg_offset = np.mean(offsets, axis=0)
        else:
            avg_offset = np.array((0.0, 0.1, 0.0))
        tail = R.dot(2 * avg_offset) + pos

        offsets = [cb.T[:3, 3] for cb in body.child_bodynodes]
        if len(offsets) > 0:
            tail = np.mean(offsets, axis=0)
            if np.allclose(head, tail):
                tail = head + (0.0, 0.0, 0.1)
        else:
            tail = head + (0.0, 0.0, 0.1)

        bone.tail = tail
        print("\ttail = %s" % str(tail))

    bpy.ops.object.mode_set(mode='OBJECT')

    for body in skel.bodynodes:
        if body.name not in ob.data.bones:
            print("bone [%s] is not created" % body.name)

    print("create_bones..... OK")
    return ob


# mesh = bpy.data.objects["[CapsuleShape(4:0)]"]
# arma = bpy.data.objects["0007.vsk"]
# bone = arma.data.bones["RightUpLeg"]

# bpy.ops.object.select_all(action='DESELECT')  # deselect all object
# arma.select = True
# bpy.ops.object.mode_set(mode='POSE')

# # bpy.ops.object.select_all(action='DESELECT')  # deselect all object
# mesh.select = True
# bone.select = True
# # bpy.context.scene.objects.active = mesh
# arma.data.bones.active = bone
# bpy.ops.object.parent_set(type='BONE', keep_transform=True)



#
# T_obj = obj.matrix_world
# print(T_obj)
# T_bone = bone.matrix
# print(T_bone)
# T = np.linalg.inv(T_bone).dot(T_obj)
# obj.matrix_local = T

# obj.parent = arma
# obj.parent_bone = "RightUpLeg"
# obj.parent_type = 'BONE'

# bpy.ops.object.select_all(action='DESELECT')  # deselect all object
# arma = bpy.data.objects["0007.vsk"]
# mesh = bpy.data.objects["[CapsuleShape(4:0)]"]

# arma.select = True
# bpy.ops.object.mode_set(mode='POSE')
# arma.data.bones["RightUpLeg"].select = True
# mesh.select = True
# bpy.context.scene.objects.active = mesh
# bpy.ops.object.parent_set(type='BONE', keep_transform=False)

# bpy.ops.object.mode_set(mode='OBJECT')
