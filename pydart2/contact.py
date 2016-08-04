# Copyright (c) 2015, Disney Research
# All rights reserved.
#
# Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
# Disney Research Robotics Group
import pydart2_api as papi


class Contact(object):
    """
    """
    def __init__(self, _state):
        """
        """
        self.state = _state
        self.point = _state[:3]
        self.force = _state[3:6]

    @property
    def p(self, ):
        return self.point

    @property
    def f(self, ):
        return self.force

    def render(self, size, scale):
        papi.collisionresult__renderContact(self.state, size, scale)

    def __repr__(self, ):
        return "[Contact %s %s]" % (self.point, self.force)
