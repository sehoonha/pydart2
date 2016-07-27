# Copyright (c) 2015, Disney Research
# All rights reserved.
#
# Author(s): Sehoon Ha <sehoon.ha@disneyresearch.com>
# Disney Research Robotics Group


import pydart2_api as papi
import numpy as np
from skeleton import Skeleton


class World(object):
    NUM_COLLISION_DETECTOR_TYPES = 4
    DART_COLLISION_DETECTOR, \
        FCL_COLLISION_DETECTOR, \
        FCL_MESH_COLLISION_DETECTOR, \
        BULLET_COLLISION_DETECTOR = range(NUM_COLLISION_DETECTOR_TYPES)

    def __init__(self, step, skel_path=None):
        self.skeletons = list()
        self.control_skel = None
        if skel_path is not None:
            self.id = papi.createWorldFromSkel(skel_path)
            self.set_time_step(step)
            nskels = self.num_skeletons()
            for i in range(nskels):
                self.add_skeleton_from_id(i)
        else:
            self.id = papi.createWorld(step)

        self.reset()

    def destroy(self):
        papi.destroyWorld(self.id)

    def add_skeleton(self, filename):
        # self.skels += [Skeleton(self, filename, friction,
        #                         _traditional=traditional)]
        # if control:
        #     self.control_skel = self.skels[-1]
        pass

    def add_skeleton_from_id(self, _skel_id):
        skel = Skeleton(_world=self, _id=_skel_id)
        self.skeletons.append(skel)
        return skel

    def num_skeletons(self):
        return papi.world__getNumSkeletons(self.id)

    @property
    def skel(self):
        """ returns the default control skeleton """
        return self.control_skel

    def time(self):
        return papi.world__getTime(self.id)

    @property
    def t(self):
        return self.time()

    def time_step(self):
        return papi.world__getTimeStep(self.id)

    @property
    def frame(self):
        return self._frame

    @property
    def dt(self):
        return self.time_step()

    def set_time_step(self, _time_step):
        papi.world__setTimeStep(self.id, _time_step)

    @dt.setter
    def dt(self, _dt):
        self.set_time_step(_dt)

    def num_frames(self):
        return papi.world__getSimFrames(self.id)

    @property
    def nframes(self):
        return self.num_frames()

    # def set_collision_detector(self, detector_type):
    #     papi.setCollisionDetector(self.id, detector_type)

    # def print_collision_detector(self):
    #     papi.printCollisionDetector(self.id)

    # def check_collision(self, checkAll=1):
    #     papi.checkCollisionWorld(self.id, checkAll)

    # def generated_contacts(self):
    #     n = papi.getWorldNumContacts(self.id)
    #     contacts = papi.getWorldContacts(self.id, 7 * n)
    #     return [Contact(contacts[7 * i: 7 * (i + 1)]) for i in range(n)]

    # def contacts(self):
    #     if self.frame < 0 or self.frame >= len(self.contact_history):
    #         return []
    #     return self.contact_history[self._frame]

    def reset(self):
        papi.world__reset(self.id)
        self._frame = 0
        self.contact_history = []
        self.contact_history.append([])  # For the initial frame

    def step(self):
        for skel in self.skeletons:
            if skel.controller is not None:
                skel.tau = skel.controller.compute()

        papi.world__step(self.id)
        self._frame += 1
        # self.contact_history.append(self.generated_contacts())

    # def set_frame(self, i):
    #     self._frame = i
    #     papi.setWorldSimFrame(self.id, i)

    def render(self):
        papi.world__render(self.id)
        # for skel in self.skels:
        #     skel.render_markers()

    # def states(self):
    #     return np.concatenate([skel.x for skel in self.skels])

    # @property
    # def x(self):
    #     return self.states()

    # def set_states(self, _x):
    #     lo = 0
    #     for skel in self.skels:
    #         hi = lo + 2 * skel.ndofs
    #         skel.x = _x[lo:hi]
    #         lo = hi

    # @x.setter
    # def x(self, _x):
    #     self.set_states(_x)

    # def save(self, filename):
    #     return papi.saveWorldToFile(self.id, filename)

    # def set_collision_pair(self, body1, body2, is_enable):
    #     flag_enable = 1 if is_enable else 0
    #     papi.setWorldCollisionPair(self.id,
    #                                body1.skel.id, body1.id,
    #                                body2.skel.id, body2.id,
    #                                flag_enable)

    def __repr__(self):
        return "[World.%d %.4f]" % (self.id, self.t)
