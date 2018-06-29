import pydart2
import pydart2.blender.blender as blender

pydart2.init()
print("pydart2 initialized")
filename = "/home/sehoonha/dev/pydart2/examples/data/vsk/0007.vsk"
blender.load_skeleton(filename, 2)
# filename = "/home/sehoonha/dev/pydart2/examples/data/sdf/atlas/atlas_v3_no_head.sdf"
# blender.load_skeleton(filename)

# def load_skeleton(filename):
#     pass


# def create_meshes():
#     import pydart2.utils.transformations as trans
#     import pydart2 as pydart
#     import pydart2.shape as ps
#     pydart.init()
#     world = pydart.world.World(0.001)
#     world.set_collision_detector(2)
#     filename = "/home/sehoonha/dev/pydart2/examples/data/vsk/0007.vsk"
#     # filename = "/home/sehoonha/dev/pydart2/examples/data/sdf/atlas/atlas_v3_no_head.sdf"
#     print("add_skeleton [%s]" % filename)
#     skel = world.add_skeleton(filename)
#     print("skeleton load OK: [%s]" % str(skel))

#     R_x = trans.rotation_matrix(0.5 * np.pi, direction=(1.0, 0.0, 0.0))
#     for body in skel.bodynodes:
#         print()
#         print(">>> %s <<<" % str(body))
#         T_b = body.world_transform()
#         for shnode in body.shapenodes:
#             if not shnode.has_visual_aspect():
#                 continue
#             shape = shnode.shape
#             rgba = shnode.visual_aspect_rgba()
#             shape_name = str(shape)
#             T_s = shnode.relative_transform()
#             print("\t%s: rgba = %s" % (str(shape), str(rgba)))

#             obj = None
#             if isinstance(shape, ps.CapsuleShape):
#                 r = shape.radius()
#                 h = shape.height()
#                 obj = create_capsule(shape_name, r, h, rgba[:3])
#                 print("create_capsule", shape_name, r, h, rgba)
#                 T_t = trans.translation_matrix((0.0, 0.0, -0.5 * h))
#             elif isinstance(shape, ps.BoxShape):
#                 sz = shape.size()
#                 obj = create_box(shape_name, sz, rgba[:3])
#                 T_t = np.identity(4)
#                 print("create_box", shape_name, sz, rgba)
#             elif isinstance(shape, ps.MeshShape):
#                 filename = shape.path()
#                 filename = filename.replace(".dae", ".stl")
#                 # obj = import_obj(shape_name, filename, rgba[:3])
#                 obj = import_stl(shape_name, filename, rgba[:3])
#                 T_t = np.identity(4)
#                 print("import_stl", shape_name, filename, rgba)

#             if obj is not None:
#                 # T = T_b.dot(R_x).dot(T_s)
#                 T = T_b.dot(T_s)
#                 # T = T_b.dot(T_s).dot(R_x)
#                 T = T_b.dot(T_s).dot(T_t)
#                 print(T_b)
#                 print(T_s)
#                 print(R_x)
#                 set_object_transformation(obj, T)


# def test():
#     import pydart2 as pydart
#     import pydart2.utils.transformations as trans
#     import os.path
#     # import pydart2.utils.transformations as trans
#     # import pydart2.shape as ps
#     pydart.init()
#     world = pydart.world.World(0.001)
#     world.set_collision_detector(2)
#     filename = "/home/sehoonha/dev/pydart2/examples/data/vsk/0007.vsk"
#     print("add_skeleton [%s]" % filename)
#     skel = world.add_skeleton(filename)
#     print("skeleton load OK: [%s]" % str(skel))

#     bpy.ops.object.add(
#         type='ARMATURE',
#         enter_editmode=True,
#         location=(0.0, 0.0, 0.0))
#     ob = bpy.context.object
#     ob.show_x_ray = True
#     ob.name = os.path.basename(filename)
#     amt = ob.data
#     amt.name = ob.name + 'Amt'
#     amt.show_axes = True

#     # Create bones
#     bpy.ops.object.mode_set(mode='EDIT')

#     for body in skel.bodynodes:
#         print(body)
#         bone = amt.edit_bones.new(body.name)

#         T = body.world_transform()
#         R = T[:3, :3]
#         pos = T[:3, 3]
#         bone.head = pos
#         # bone.rotation_quaternion = trans.quaternion_from_matrix(T)
#         # bone.matrix = T

#         if body.parent_bodynode is not None:
#             bone.parent = amt.edit_bones[body.parent_bodynode.name]
#             bone.use_connect = False

#         offsets = [sh.offset() for sh in body.shapenodes
#                    if sh.has_visual_aspect()]
#         if len(offsets) > 0:
#             avg_offset = np.mean(offsets, axis=0)
#             bone_length = np.fabs(avg_offset[1]) * 2.0
#         else:
#             bone_length = 0.05

#         # tail = R.dot(np.array((0.0, bone_length, 0.0))) + pos
#         tail = R.dot(2 * avg_offset) + pos
#         bone.tail = tail

#     bpy.ops.object.mode_set(mode='OBJECT')
#     return ob


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

# #
# # T_obj = obj.matrix_world
# # print(T_obj)
# # T_bone = bone.matrix
# # print(T_bone)
# # T = np.linalg.inv(T_bone).dot(T_obj)
# # obj.matrix_local = T

# # obj.parent = arma
# # obj.parent_bone = "RightUpLeg"
# # obj.parent_type = 'BONE'

# # bpy.ops.object.select_all(action='DESELECT')  # deselect all object
# # arma = bpy.data.objects["0007.vsk"]
# # mesh = bpy.data.objects["[CapsuleShape(4:0)]"]

# # arma.select = True
# # bpy.ops.object.mode_set(mode='POSE')
# # arma.data.bones["RightUpLeg"].select = True
# # mesh.select = True
# # bpy.context.scene.objects.active = mesh
# # bpy.ops.object.parent_set(type='BONE', keep_transform=False)

# # bpy.ops.object.mode_set(mode='OBJECT')
