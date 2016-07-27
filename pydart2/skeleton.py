# Copyright (c) 2015, Disney Research
# All rights reserved.
#
# Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
# Disney Research Robotics Group


import os.path
import pydart2_api as papi
import numpy as np
from skel_vector import SkelVector
# from body import Body
# from dof import Dof
# from marker import Marker


class Skeleton(object):
    def __init__(self, _world, _filename=None,
                 _id=None, _friction=None, ):
        self.world = _world
        self.filename = _filename
        self.friction = _friction
        if self.filename is not None:
            self.id = papi.addSkeleton(self.world.id, _filename)
        else:
            self.id = _id

        self.controller = None

        # Initialize dofs
        _ndofs = papi.skeleton__getNumDofs(self.world.id, self.id)
        self.dofs = ["DoF" for i in range(_ndofs)]
        # self.dofs = [Dof(self, i) for i in range(_ndofs)]
        # self.name_to_dof = {dof.name: dof for dof in self.dofs}

        # # Initialize bodies
        # _nbodies = papi.getSkeletonNumBodies(self.world.id, self.id)
        # self.bodies = [Body(self, i) for i in range(_nbodies)]
        # self.name_to_body = {body.name: body for body in self.bodies}
        # self.controller = None

        # # Initialize markers
        # self.markers = list()
        # for body in self.bodies:
        #     for j in range(body.num_markers()):
        #         m = Marker(body, j)
        #         self.markers.append(m)

    @property
    def name(self):
        return papi.skeleton__getName(self.world.id, self.id)

    # def set_joint_damping(self, _damping):
    #     papi.setSkeletonJointDamping(self.world.id, self.id, _damping)

    def num_dofs(self):
        return len(self.dofs)

    @property
    def ndofs(self):
        return self.num_dofs()

    # def num_bodies(self):
    #     return len(self.bodies)

    # @property
    # def nbodies(self):
    #     return self.num_bodies()

    def mass(self):
        return papi.skeleton__getMass(self.world.id, self.id)

    @property
    def m(self):
        return self.mass()

    # def mass_matrix(self):
    #     M = np.zeros((self.ndofs, self.ndofs))
    #     papi.getSkeletonMassMatrix(self.world.id, self.id, M)
    #     return M

    # @property
    # def M(self):
    #     return self.mass_matrix()

    def positions(self):
        q = papi.skeleton__getPositions(self.world.id, self.id, self.ndofs)
        return q
        # return SkelVector(q, self)

    @property
    def q(self):
        return self.positions()

    def set_positions(self, _q):
        papi.skeleton__setPositions(self.world.id, self.id, _q)

    @q.setter
    def q(self, _q):
        """ Setter also updates the internal skeleton kinematics """
        self.set_positions(_q)

    # def position_lower_limit(self):
    #     return papi.getSkeletonPositionLowerLimit(self.world.id,
    #                                               self.id, self.ndofs)

    # def position_upper_limit(self):
    #     return papi.getSkeletonPositionUpperLimit(self.world.id,
    #                                               self.id, self.ndofs)

    # @property
    # def q_lo(self):
    #     return self.position_lower_limit()

    # @property
    # def q_hi(self):
    #     return self.position_upper_limit()

    # def velocities(self):
    #     qdot = papi.getSkeletonVelocities(self.world.id, self.id, self.ndofs)
    #     return SkelVector(qdot, self)

    # @property
    # def qdot(self):
    #     return self.velocities()

    # def set_velocities(self, _qdot):
    #     papi.setSkeletonVelocities(self.world.id, self.id, _qdot)

    # @qdot.setter
    # def qdot(self, _qdot):
    #     """ Setter also updates the internal skeleton kinematics """
    #     self.set_velocities(_qdot)

    # def states(self):
    #     return np.concatenate((self.positions(), self.velocities()))

    # def position_differences(self, q1, q2):
    #     ret = papi.getSkeletonPositionDifferences(self.world.id, self.id,
    #                                               q1, q2, self.ndofs)
    #     return ret

    # def velocity_differences(self, q1, q2):
    #     ret = papi.getSkeletonVelocityDifferences(self.world.id, self.id,
    #                                               q1, q2, self.ndofs)
    #     return ret

    # @property
    # def x(self):
    #     return np.concatenate((self.positions(), self.velocities()))

    # def set_states(self, _x):
    #     self.set_positions(_x[:self.ndofs])
    #     self.set_velocities(_x[self.ndofs:])

    # @x.setter
    # def x(self, _x):
    #     self.set_states(_x)

    # def coriolis_and_gravity_forces(self):
    #     return papi.getSkeletonCoriolisAndGravityForces(self.world.id,
    #                                                     self.id, self.ndofs)

    def is_mobile(self):
        return papi.skeleton__isMobile(self.world.id, self.id)

    def set_mobile(self, mobile):
        papi.skeleton__getMobile(self.world.id, self.id, mobile)

    # def set_self_collision(self, self_col, adj_col):
    #     flag_self = 1 if self_col is True else 0
    #     flag_adj = 1 if adj_col is True else 0
    #     papi.setSkeletonSelfCollision(self.world.id, self.id,
    #                                   flag_self, flag_adj)

    # def remove_all_collision_pairs(self):
    #     for b1 in self.bodies:
    #         for b2 in self.bodies:
    #             self.world.set_collision_pair(b1, b2, False)

    # @property
    # def c(self):
    #     return self.coriolis_and_gravity_forces()

    # def constraint_forces(self):
    #     return papi.getSkeletonConstraintForces(self.world.id,
    #                                             self.id, self.ndofs)

    # def body(self, query):
    #     if isinstance(query, str):
    #         return self.name_to_body[query]
    #     elif isinstance(query, int):
    #         return self.bodies[query]
    #     else:
    #         print 'No find...', query
    #         return None

    # def body_index(self, _name):
    #     return self.name_to_body[_name].id

    # def dof(self, query):
    #     if isinstance(query, str):
    #         return self.name_to_dof[query]
    #     elif isinstance(query, int):
    #         return self.dofs[query]
    #     else:
    #         print 'No find...', query
    #         return None

    # def dof_index(self, _name):
    #     return self.name_to_dof[_name].id

    # def dof_indices(self, _names):
    #     return np.array([self.dof_index(n) for n in _names])

    # def world_com(self):
    #     return papi.getSkeletonWorldCOM(self.world.id, self.id)

    # @property
    # def C(self):
    #     return self.world_com()

    # @property
    # def COM(self):
    #     return self.world_com()

    # def world_com_velocity(self):
    #     return papi.getSkeletonWorldCOMVelocity(self.world.id, self.id)

    # @property
    # def Cdot(self):
    #     return self.world_com_velocity()

    # def linear_momentum(self):
    #     return self.Cdot * self.m

    # @property
    # def P(self):
    #     return self.linear_momentum()

    # def forces(self):
    #     return self._tau

    # @property
    # def tau(self):
    #     return self.forces()

    # def set_forces(self, _tau):
    #     self._tau = _tau
    #     papi.setSkeletonForces(self.world.id, self.id, _tau)

    # @tau.setter
    # def tau(self, _tau):
    #     self.set_forces(_tau)

    # def force_lower_limit(self):
    #     return papi.getSkeletonForceLowerLimit(self.world.id,
    #                                            self.id, self.ndofs)

    # def force_upper_limit(self):
    #     return papi.getSkeletonForceUpperLimit(self.world.id,
    #                                            self.id, self.ndofs)

    # @property
    # def tau_lo(self):
    #     return self.force_lower_limit()

    # @property
    # def tau_hi(self):
    #     return self.force_upper_limit()

    # def approx_inertia(self, axis):
    #     """Calculates the point-masses approximated inertia
    #     with respect to the given axis """
    #     axis = np.array(axis) / np.linalg.norm(axis)
    #     I = 0
    #     C = self.C
    #     for body in self.bodies:
    #         d = body.C - C
    #         # Subtract the distance along the axis
    #         r_sq = np.linalg.norm(d) ** 2 - np.linalg.norm(d.dot(axis)) ** 2
    #         I += body.m * r_sq
    #     return I

    # def approx_inertia_x(self):
    #     return self.approx_inertia([1, 0, 0])

    # def approx_inertia_y(self):
    #     return self.approx_inertia([0, 1, 0])

    # def approx_inertia_z(self):
    #     return self.approx_inertia([0, 0, 1])

    # def external_contacts_and_body_id(self):
    #     cid_cnt = dict()
    #     contacts = []
    #     for body in self.bodies:
    #         for c in body.contacts():
    #             contacts += [(c, body.id)]
    #             cid = int(c[6])
    #             if cid not in cid_cnt:
    #                 cid_cnt[cid] = 1
    #             else:
    #                 cid_cnt[cid] += 1
    #     return [(c, bid) for (c, bid) in contacts if cid_cnt[int(c[6])] < 2]

    # def contact_id_set(self):
    #     id_list = []
    #     for b in self.bodies:
    #         id_list += [c.i for c in b.contacts()]
    #     return set(id_list)

    # def contacted_bodies(self):
    #     return [body for body in self.bodies if body.num_contacts() > 0]

    # def world_cop(self):
    #     bodies = self.contacted_bodies()
    #     if len(bodies) == 0:
    #         return None
    #     pos_list = [b.C for b in bodies]
    #     avg = sum(pos_list) / len(pos_list)
    #     return avg

    # @property
    # def COP(self):
    #     return self.world_cop()

    # def contacted_body_names(self):
    #     return [body.name for body in self.contacted_bodies()]

    def render(self):
        papi.renderSkeleton(self.world.id, self.id)

    def render_with_color(self, color):
        if len(color) == 3:
            color = np.concatenate([color, [1.0]])
        papi.renderSkeletonWithColor(self.world.id, self.id, color)

    # def render_markers(self):
    #     papi.renderSkeletonMarkers(self.world.id, self.id)

    def __repr__(self):
        return '[Skeleton.%d.%s]' % (self.id, os.path.basename(self.filename))
