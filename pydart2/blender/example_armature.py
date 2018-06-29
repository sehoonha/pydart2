#---------------------------------------------------
# File armature.py
#---------------------------------------------------
import bpy
import math
from mathutils import Vector, Matrix


def createRig(name, origin, boneTable):
    # Create armature and object
    bpy.ops.object.add(
        type='ARMATURE',
        enter_editmode=True,
        location=origin)
    ob = bpy.context.object
    ob.show_x_ray = True
    ob.name = name
    amt = ob.data
    amt.name = name + 'Amt'
    amt.show_axes = True

    # Create bones
    bpy.ops.object.mode_set(mode='EDIT')
    for (bname, pname, vector) in boneTable:
        bone = amt.edit_bones.new(bname)
        if pname:
            parent = amt.edit_bones[pname]
            bone.parent = parent
            bone.head = parent.tail
            bone.use_connect = False
            (trans, rot, scale) = parent.matrix.decompose()
        else:
            bone.head = (0, 0, 0)
            rot = Matrix.Translation((0, 0, 0))   # identity matrix
        bone.tail = rot * Vector(vector) + bone.head
    bpy.ops.object.mode_set(mode='OBJECT')
    return ob


def poseRig(ob, poseTable):
    bpy.context.scene.objects.active = ob
    bpy.ops.object.mode_set(mode='POSE')

    for (bname, axis, angle) in poseTable:
        pbone = ob.pose.bones[bname]
        # Set rotation mode to Euler XYZ, easier to understand
        # than default quaternions
        pbone.rotation_mode = 'XYZ'
        # Documentation bug: Euler.rotate(angle,axis):
        # axis in ['x','y','z'] and not ['X','Y','Z']
        pbone.rotation_euler.rotate_axis(axis, math.radians(angle))
    bpy.ops.object.mode_set(mode='OBJECT')


def run(origo):
    origin = Vector(origo)
    # Table of bones in the form (bone, parent, vector)
    # The vector is given in local coordinates
    boneTable1 = [
        ('Base', None, (1, 0, 0)),
        ('Mid', 'Base', (1, 0, 0)),
        ('Tip', 'Mid', (0, 0, 1))
    ]
    bent = createRig('Bent', origin, boneTable1)

    # The second rig is a straight line, i.e. bones run along local Y axis
    boneTable2 = [
        ('Base', None, (1, 0, 0)),
        ('Mid', 'Base', (0, 0.5, 0)),
        ('Mid2', 'Mid', (0, 0.5, 0)),
        ('Tip', 'Mid2', (0, 1, 0))
    ]
    straight = createRig('Straight', origin + Vector((0, 2, 0)), boneTable2)

    # Pose second rig
    poseTable2 = [
        ('Base', 'X', 90),
        ('Mid2', 'Z', 45),
        ('Tip', 'Y', -45)
    ]
    poseRig(straight, poseTable2)

    # Pose first rig
    poseTable1 = [
        ('Tip', 'Y', 45),
        ('Mid', 'Y', 45),
        ('Base', 'Y', 45)
    ]
    poseRig(bent, poseTable1)


if __name__ == "__main__":
    run((0, 5, 0))
