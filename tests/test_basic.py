import unittest
import pydart2 as pydart
from tests.config import DATA_DIR


class TestBasic(unittest.TestCase):
    def setUp(self):
        pydart.init(verbose=False)

    def test_world_create(self):
        world = pydart.world.World(0.001)
        self.assertIsNotNone(world)
        self.assertEqual(world.id, 0)

    def test_world_from_skeleton(self, ):
        world = pydart.world.World(
            0.001,
            DATA_DIR + "skel/cubes.skel"
            )
        self.assertIsNotNone(world)
        self.assertEqual(world.id, 0)

    def test_cube_collisions(self, ):
        world = pydart.world.World(
            0.001,
            DATA_DIR + "skel/cubes.skel"
            )
        self.assertIsNotNone(world)
        self.assertEqual(world.id, 0)

        skel = world.skeletons[-1]
        while world.t < 3.0:
            world.step()
        # target = [7.94625426e-02, -3.25000010e-01, -9.22240587e-18]
        # self.assertAlmostEqual(skel.C[0], target[0])
        # self.assertAlmostEqual(skel.C[1], target[1])
        # self.assertAlmostEqual(skel.C[2], target[2])
        self.assertLess(0.0, skel.C[0])
