from . import pydart2_api as papi


class IK(papi.Pydart2IK):
    """docstring for IK"""
    def __init__(self, ):
        super(IK, self).__init__()
        self.world = None
        self.skel = None

    def set_skeleton(self, skel):
        super(IK, self).set_skeleton(skel.world.id, skel.id)
        self.world = skel.world
        self.skel = skel

    def add_body_and_offset(self, body, offset):
        args = (self.world.id, self.skel.id, body.id)
        super(IK, self).add_body_and_offset(*args, offset)
