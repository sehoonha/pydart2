# Copyright (c) 2015, Disney Research
# All rights reserved.
#
# Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
# Disney Research Robotics Group
import pydart2_api as papi


class Joint(object):
    """
    """
    def __init__(self, _skeleton, _id):
        """
        """
        self.skeleton = _skeleton
        self.id = _id
        self.name = papi.joint__getName(self.wid, self.skid, self.id)
        self.parent_bodynode = None
        self.child_bodynode = None
        self.dofs = list()

    @property
    def skel(self):
        return self.skeleton

    @property
    def wid(self):
        return self.skel.world.id

    @property
    def skid(self):
        return self.skel.id

    def build(self):
        skel = self.skel
        self.parent_bodynode = None
        self.child_bodynode = None

        parent_id = self.parent_body_node_id()
        if parent_id >= 0:
            self.parent_bodynode = skel.bodynodes[
                self.parent_body_node_id()]

        child_id = self.child_body_node_id()
        if child_id >= 0:
            self.child_bodynode = skel.bodynodes[
                self.child_body_node_id()]

        self.dofs = list()
        _ndofs = papi.joint__getNumDofs(self.wid, self.skid, self.id)
        for i in range(_ndofs):
            id = papi.joint__getDof(self.wid, self.skid, self.id, i)
            self.dofs.append(skel.dofs[id])
            skel.dofs[id].joint = self

########################################
# Joint::Property Functions
    def set_name(self, _name, _renameDofs):
        return papi.joint__setName(self.wid,
                                   self.skid,
                                   self.id,
                                   _name,
                                   _renameDofs)

    def is_kinematic(self, ):
        return papi.joint__isKinematic(self.wid, self.skid, self.id)

    def is_dynamic(self, ):
        return papi.joint__isDynamic(self.wid, self.skid, self.id)

########################################
# Joint::Parent and child functions
    def parent_body_node_id(self, ):
        return papi.joint__getParentBodyNode(self.wid, self.skid, self.id)

    def child_body_node_id(self, ):
        return papi.joint__getChildBodyNode(self.wid, self.skid, self.id)

    def set_transform_from_parent_body_node(self, T):
        papi.joint__setTransformFromParentBodyNode(self.wid,
                                                   self.skid,
                                                   self.id,
                                                   T)

    def set_transform_from_child_body_node(self, T):
        papi.joint__setTransformFromChildBodyNode(self.wid,
                                                  self.skid,
                                                  self.id,
                                                  T)

    def transform_from_parent_body_node(self, ):
        return papi.joint__getTransformFromParentBodyNode(self.wid,
                                                          self.skid,
                                                          self.id)

    def transform_from_child_body_node(self, ):
        return papi.joint__getTransformFromChildBodyNode(self.wid,
                                                         self.skid,
                                                         self.id)

########################################
# Joint::Dof functions
    def num_dofs(self, ):
        return len(self.dofs)

    def __repr__(self):
        return '[Joint(%d): %s]' % (self.id, self.name)
