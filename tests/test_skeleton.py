import unittest
import pydart2 as pydart
from tests.config import DATA_DIR


class TestSkeleton(unittest.TestCase):
    def setUp(self):
        pydart.init(verbose=False)

    def test_atlas_sdf(self):
        world = pydart.world.World(0.001)
        self.assertEqual(world.id, 0)

        skel = world.add_skeleton(
            DATA_DIR + "sdf/atlas/atlas_v3_no_head.sdf")
        self.assertEqual(len(skel.bodynodes), 28)
        self.assertEqual(len(skel.joints), 28)
        self.assertEqual(len(skel.dofs), 33)

    def test_atlas_urdf(self):
        world = pydart.world.World(0.001)
        self.assertEqual(world.id, 0)

        skel = world.add_skeleton(
            DATA_DIR + "sdf/atlas/atlas_v3_no_head.urdf")
        self.assertEqual(len(skel.bodynodes), 34)
        self.assertEqual(len(skel.joints), 34)
        self.assertEqual(len(skel.dofs), 33)

    def test_puppy_sdf(self):
        world = pydart.world.World(0.001)
        self.assertEqual(world.id, 0)

        skel = world.add_skeleton(DATA_DIR + "sdf/puppy.sdf")
        self.assertEqual(len(skel.bodynodes), 15)
        self.assertEqual(len(skel.joints), 15)
        self.assertEqual(len(skel.dofs), 20)
