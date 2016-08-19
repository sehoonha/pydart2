import pydart2_api as papi
import world
import os.path
import sys
try:
    import gui
except Exception:
    e = sys.exc_info()[1]
    print("-" * 40)
    print("Error while importing pydart2.gui")
    print(e)
    print("-" * 40)

import utils
from world import World


def init(verbose=True):
    papi.init(verbose)
