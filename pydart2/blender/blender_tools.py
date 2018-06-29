import numpy as np
import bpy


def set_color(obj, color):
    if color is not None:
        mat = bpy.data.materials.new(name=obj.name + "_mat")
        mat.diffuse_color = color
        if len(obj.data.materials):
            obj.data.materials[0] = mat
        else:
            obj.data.materials.append(mat)


def create_ellipsoid(name, size, color=None, location=None):
    bpy.ops.mesh.primitive_uv_sphere_add(segments=64, ring_count=32, size=1.0)
    obj = bpy.context.object
    obj.name = name
    if isinstance(size, float):
        size = (size, size, size)
    obj.scale = size
    if location is not None:
        obj.location = location
    set_color(obj, color)
    return obj


def create_capsule(name, r, h, color=None):
    bpy.ops.mesh.primitive_cylinder_add(radius=1.0, depth=1.0)
    obj0 = bpy.context.object
    obj0.name = name + "cyl"
    obj0.scale = [r, r, h]
    obj1 = create_ellipsoid(name + "top", r, None, (0.0, 0.0, 0.5 * h))
    obj2 = create_ellipsoid(name + "btm", r, None, (0.0, 0.0, -0.5 * h))

    bpy.ops.object.select_all(action='DESELECT')  # deselect all object
    obj0.select = True
    obj1.select = True
    obj2.select = True
    bpy.ops.object.join()

    obj = bpy.context.object
    obj.name = name
    set_color(obj, color)
    return obj


def create_box(name, size, color=None):
    bpy.ops.mesh.primitive_cube_add(radius=0.5)
    obj = bpy.context.object
    obj.name = name
    obj.scale = size
    set_color(obj, color)
    return obj


def import_obj(name, filepath, color=None):
    bpy.ops.import_scene.obj(filepath=filepath)
    obj = bpy.context.selected_objects[0]
    obj.name = name
    set_color(obj, color)
    return obj


def import_stl(name, filepath, color=None):
    bpy.ops.import_mesh.stl(filepath=filepath)
    obj = bpy.context.selected_objects[0]
    obj.name = name
    set_color(obj, color)
    return obj


def set_object_transformation(obj, T):
    import pydart2.utils.transformations as trans
    txyz = trans.translation_from_matrix(T)
    # rxyz = trans.euler_from_matrix(T, axes="rxyz")
    quat = trans.quaternion_from_matrix(T)
    obj.location = txyz
    # obj.rotation_euler = rxyz
    obj.rotation_mode = 'QUATERNION'
    obj.rotation_quaternion = quat


def attach_to_bone(obj_name,
                   bone_name,
                   location=None,
                   rpy=None):
    obj = bpy.data.objects[obj_name]
    arma = bpy.data.objects["0007.vsk"]

    if location is not None:
        obj.location = location
    if rpy is not None:
        obj.rotation_euler = rpy
    obj.parent = arma
    obj.parent_bone = bone_name
    obj.parent_type = 'BONE'


def attach_to_bone2(mesh, arma, bone):
    bpy.ops.object.select_all(action='DESELECT')  # deselect all object
    arma.select = True
    bpy.ops.object.mode_set(mode='POSE')

    # bpy.ops.object.select_all(action='DESELECT')  # deselect all object
    mesh.select = True
    bone.select = True
    # bpy.context.scene.objects.active = mesh
    arma.data.bones.active = bone
    bpy.ops.object.parent_set(type='BONE', keep_transform=True)
    bpy.ops.object.mode_set(mode='OBJECT')
