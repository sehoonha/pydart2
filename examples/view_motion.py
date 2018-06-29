import pydart2 as pydart
import numpy as np
from scipy.optimize import minimize


class MyWorld(pydart.World):

    def __init__(self, vsk_filename, motion_filename):
        pydart.World.__init__(self, 0.001)
        print('pydart create_world OK')
        self.set_collision_detector(2)
        self.add_skeleton("./data/urdf/ground.urdf")
        self.robot = self.add_skeleton(vsk_filename)
        self.robot.set_root_joint_to_trans_and_euler()
        print('add_skeleton OK: %s' % vsk_filename)
        print("\t# dofs = %d" % len(self.robot.dofs))
        print("\t# markers = %d" % len(self.robot.markers))

        # # modify the foot orientations
        # trans = pydart.utils.transformations
        # for body in [self.robot.bodynode("LeftFoot"),
        #              self.robot.bodynode("RightFoot"), ]:
        #     for node in body.shapenodes:
        #         T = node.relative_transform()
        #         T[1, 3] = -0.01
        #         R = trans.rotation_matrix(10.0 / 180.0 * np.pi,
        #                                   (0, 1, 0))
        #         T = T.dot(R)
        #         node.set_relative_transform(T)

        # initialize the motion
        self.motions = np.loadtxt(motion_filename)
        print("load_motion OK: %s" % motion_filename)
        print("\tshape = %s" % str(self.motions.shape))

        self.set_frame(0)
        # self.solve()

    def num_frames(self, ):
        return len(self.motions)

    def set_frame(self, frame_index):
        self.frame_index = frame_index
        if frame_index < len(self.motions):
            self.robot.set_positions(self.motions[frame_index])

    def render_with_ri(self, ri):
        ri.set_line_width(2.0)
        ri.render_axes((0.0, 0.0, 0.0), 1.0)
        ri.set_line_width(1.0)

    def status(self, ):
        ret = str(self)
        ret += "[Frame: %d/%d]" % (self.frame_index, self.num_frames())
        return ret


if __name__ == '__main__':
    import sys
    print('Example: process_mocap.py [.vsk] [.txt (numpy)]')
    _, vsk_filename, motion_filename = sys.argv

    pydart.init()
    print('pydart initialization OK')
    world = MyWorld(vsk_filename, motion_filename)

    win = pydart.gui.pyqt5.window.PyQt5Window(world)
    win.scene.set_camera(1)  # Z-up Camera
    win.run()
