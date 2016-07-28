# Copyright (c) 2015, Disney Research
# All rights reserved.
#
# Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
# Disney Research Robotics Group
import pydart2_api as papi


class Dof(object):
    """
    """
    def __init__(self, _skel, _id):
        """
        """
        self.skel = _skel
        self.id = _id
        self.name = papi.dof__getName(self.wid, self.sid, self.id)

    @property
    def wid(self):
        return self.skel.world.id

    @property
    def sid(self):
        return self.skel.id

    def __repr__(self):
        return '[Dof(%d): %s]' % (self.id, self.name)
