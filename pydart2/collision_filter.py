from . import pydart2_api as papi


class CollisionFilter(object):
    def __init__(self, world):
        self.world = world
        self.build()

    @property
    def id(self, ):
        return self.world.id

    def build(self, ):
        papi.world__createCollisionFilter(self.id)

    def add_to_black_list(self, body0, body1):
        b0, b1 = body0, body1
        papi.world__addBodyNodePairToCollisionBlackList(self.id,
                                                        b0.skid, b0.id,
                                                        b1.skid, b1.id)

    def remove_from_black_list(self, body0, body1):
        b0, b1 = body0, body1
        papi.world__removeBodyNodePairFromCollisionBlackList(self.id,
                                                             b0.skid, b0.id,
                                                             b1.skid, b1.id)

    def remove_all_from_black_list(self, ):
        papi.world__removeAllBodyNodePairsFromCollisionBlackList(self.id)
