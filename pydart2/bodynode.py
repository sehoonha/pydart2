# Copyright (c) 2015, Disney Research
# All rights reserved.
#
# Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
# Disney Research Robotics Group
import pydart2_api as papi
import numpy as np


class BodyNode(object):
    def __init__(self, _skel, _id):
        self.skel = _skel
        self._id = _id
        self.name = papi.bodynode__getName(self.wid, self.sid, self.id)
        self.parent_bodynode = None
        self.child_bodynodes = list()

    def build(self):
        self.parent_bodynode = None
        self.child_bodynodes = list()

        ret_id = papi.bodynode__getParentBodyNode(self.wid, self.sid, self.id)
        if ret_id >= 0:
            self.parent_bodynode = self.skel.bodynodes[ret_id]

        num = papi.bodynode__getNumChildBodyNodes(self.wid, self.sid, self.id)
        for index in range(num):
            ret_id = papi.bodynode__getChildBodyNode(self.wid,
                                                     self.sid,
                                                     self.id,
                                                     index)
            if ret_id >= 0:
                self.child_bodynodes.append(self.skel.bodynodes[ret_id])

    def num_child_bodynodes(self, ):
        return len(self.child_bodynodes)

    @property
    def id(self):
        return self._id

    @property
    def wid(self):
        return self.skel.world.id

    @property
    def sid(self):
        return self.skel.id

    # def num_contacts(self):
    #     return papi.getBodyNodeNumContacts(self.wid, self.sid, self.id)

    # def contacts(self):
    #     n = self.num_contacts()
    #     contacts = papi.getBodyNodeContacts(self.wid, self.sid, self.id, 7 * n)
    #     return [Contact(contacts[7 * i: 7 * (i + 1)]) for i in range(n)]

    def mass(self):
        return papi.bodynode__getMass(self.wid, self.sid, self.id)

    @property
    def m(self):
        return self.mass()

    def inertia(self):
        return papi.bodynode__getInertia(self.wid, self.sid, self.id)

    @property
    def I(self):
        return self.inertia()

    # def local_com(self):
    #     return papi.getBodyNodeLocalCOM(self.wid, self.sid, self.id)

    # def world_com(self):
    #     return papi.getBodyNodeWorldCOM(self.wid, self.sid, self.id)

    # def bounding_box_dims(self):
    #     return papi.getBodyNodeShapeBoundingBoxDim(self.wid, self.sid, self.id)

    # def to_world(self, x):
    #     x_ = np.append(x, [1.0])
    #     return (self.T.dot(x_))[:3]

    # def to_local(self, x):
    #     Tinv = np.linalg.inv(self.T)
    #     x_ = np.append(x, [1.0])
    #     return (Tinv.dot(x_))[:3]

    # @property
    # def C(self):
    #     return self.world_com()

    # def world_com_velocity(self):
    #     return papi.getBodyNodeWorldCOMVelocity(self.wid, self.sid, self.id)

    # def world_com_spatial_velocity(self):
    #     return papi.getBodyNodeWorldCOMSpatialVelocity(self.wid,
    #                                                    self.sid,
    #                                                    self.id)

    # def world_com_spatial_acceleration(self):
    #     return papi.getBodyNodeWorldCOMSpatialAcceleration(self.wid,
    #                                                        self.sid,
    #                                                        self.id)

    # def local_com_spatial_velocity(self):
    #     return papi.getBodyNodeLocalCOMSpatialVelocity(self.wid,
    #                                                    self.sid,
    #                                                    self.id)

    # def local_com_spatial_acceleration(self):
    #     return papi.getBodyNodeLocalCOMSpatialAcceleration(self.wid,
    #                                                        self.sid,
    #                                                        self.id)

    # @property
    # def Cdot(self):
    #     return self.world_com_velocity()

    # def transformation(self):
    #     return papi.getBodyNodeTransformation(self.wid, self.sid, self.id)

    # @property
    # def T(self):
    #     return self.transformation()

    # def world_linear_jacobian(self, offset=None):
    #     if offset is None:
    #         offset = np.zeros(3)
    #     J = np.zeros((3, self.skel.ndofs))
    #     papi.getBodyNodeWorldLinearJacobian(self.wid, self.sid,
    #                                         self.id, offset, J)
    #     return J

    # @property
    # def J(self, offset=None):
    #     return self.world_linear_jacobian(offset)

    # def add_ext_force(self, f):
    #     papi.addBodyNodeExtForce(self.wid, self.sid, self.id, f)

    # def add_ext_force_at(self, f, offset):
    #     papi.addBodyNodeExtForceAt(self.wid, self.sid, self.id, f, offset)

    # def enable_collision(self, rhs_body):
    #     self.skel.world.set_collision_pair(self, rhs_body, True)

    # def num_markers(self):
    #     return papi.getBodyNodeNumMarkers(self.wid, self.sid, self.id)

    # def get_marker_local_pos(self, mid):
    #     return papi.getMarkerLocalPosition(self.wid, self.sid, self.id, mid)

    # def get_marker_pos(self, mid):
    #     return papi.getMarkerPosition(self.wid, self.sid, self.id, mid)

    def __repr__(self):
        return '[BodyNode(%d): %s]' % (self.id, self.name)
