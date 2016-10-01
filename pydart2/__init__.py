from __future__ import print_function
from __future__ import absolute_import
from . import pydart2_api as papi
from . import world
from . import constraints
import os.path
import sys
try:
    from . import gui
except Exception:
    e = sys.exc_info()[1]
    print("-" * 40)
    print("Error while importing pydart2.gui")
    print(e)
    print("-" * 40)

from . import utils
from .world import World
from .skel_vector import SkelVector


__version__ = (0, 5, 0)


def init(verbose=True):
    papi.init(verbose)
