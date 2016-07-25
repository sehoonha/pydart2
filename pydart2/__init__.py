import pydart2_api as papi
import world
import gui


def boo(x):
    print("pydart2.boo is excuted")
    return x * 2


def init():
    papi.init()


def create_world(step, skel_path=None):
    return world.World(step, skel_path)
