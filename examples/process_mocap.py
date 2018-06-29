import pydart2 as pydart
import c3d  # sudo pip3 install c3d
import numpy as np
from scipy.optimize import minimize
from time import time


class MyWorld(pydart.World):

    def __init__(self, vsk_filename, c3d_filename):
        pydart.World.__init__(self, 0.001)
        print('pydart create_world OK')
        self.set_collision_detector(2)
        self.robot = self.add_skeleton(vsk_filename)
        # reconstruct the marker order
        markers_saved = self.robot.markers
        for m in markers_saved:
            m.saved_offset = m.local_position()
        self.robot.set_root_joint_to_trans_and_euler()
        print('add_skeleton OK: %s' % vsk_filename)
        print("\t# dofs = %d" % len(self.robot.dofs))
        print("\t# markers = %d" % len(self.robot.markers))
        print(self.robot.m)
        for body in self.robot.bodynodes:
            print(str(body), ":", body.m)
        for shapenode in self.robot.bodynodes[0].shapenodes:
            shapenode.set_visual_aspect_rgba((1.0, 1.0, 0.3, 1.0))

        marker_ordered = list()
        for lhs in markers_saved:
            # print("search... " + str(lhs))
            for rhs in self.robot.markers:
                # print("[%s][%s]" % (lhs.bodynode.name, rhs.bodynode.name))
                if lhs.bodynode.name != rhs.bodynode.name:
                    continue
                if not np.allclose(lhs.saved_offset,
                                   rhs.local_position()):
                    continue
                marker_ordered.append(rhs)
                self.robot.markers.remove(rhs)
                print("match %s <-- %s" % (str(lhs), str(rhs)))
                break
        assert(len(self.robot.markers) == 0)
        self.robot.markers = marker_ordered

        for m in self.robot.markers:
            print(m, m.bodynode)

        self.build_marker_indices_from_vsk(vsk_filename)

        self.c3d_filename = c3d_filename
        self.c3d_data = list()
        self.num_markers = 0
        if c3d_filename.endswith(".c3d"):
            self.load_c3d(c3d_filename)
        elif c3d_filename.endswith(".txt"):
            self.load_c3d_ascii(c3d_filename)

        print('parse C3D OK: %s' % c3d_filename)
        print('\t# frames = %d' % len(self.c3d_data))
        print('\t# markers = %d' % self.num_markers)

        # initialize the motion
        self.motions = list()

        self.set_frame(0)
        # self.solve()

    def load_c3d(self, filename):
        self.c3d_data = list()
        unit = 0.001  # assume the unit is millimeter
        axis = np.array((1, 2, 0))
        scale = np.array((1.0, 1.0, 1.0))

        with open(filename, 'rb') as handle:
            reader = c3d.Reader(handle)
            for i, points, analog in reader.read_frames():
                # print('frame {}: point {}, analog {}'.format(
                #     i, points.shape, analog.shape))
                positions = [unit * p[:3] for p in points]
                # positions = positions[-62:]
                positions = [positions[i] for i in self.marker_indices]
                positions = np.array(positions)
                positions = scale * positions[:, axis]

                self.c3d_data.append(positions)
            self.num_markers = len(positions)

    def load_c3d_ascii(self, filename):
        self.c3d_data = list()
        unit = 0.001  # assume the unit is millimeter
        axis = np.array((0, 1, 2))
        scale = np.array((1.0, 1.0, 1.0))
        with open(filename) as fin:
            for i, line in enumerate(fin.readlines()):
                tokens = line.split()
                if len(tokens) == 0 or not tokens[0].isdigit():
                    continue
                positions = unit * np.array([float(x) for x in tokens[1:]])
                positions = positions.reshape(-1, 3)
                positions = [positions[i] for i in self.marker_indices]
                positions = np.array(positions)
                positions = scale * positions[:, axis]
                self.c3d_data.append(positions)
            self.num_markers = len(positions)

    def build_marker_indices_from_vsk(self, filename):
        from bs4 import BeautifulSoup
        bodynames = set([b.name for b in self.robot.bodynodes])
        indices = list()
        print("build marker_indices from vsk file...")
        with open(filename) as fin:
            soup = BeautifulSoup(fin, 'xml')
            for i, m in enumerate(soup.find_all("Marker")):
                if m["SEGMENT"] in bodynames:
                    args = i, m["NAME"], m["SEGMENT"]
                    print("\t%dth marker %s at %s" % args)
                    indices.append(i)
                else:
                    args = i, m["NAME"], m["SEGMENT"]
                    print("..skip %dth %s at %s" % args)
        self.marker_indices = indices
        print("build marker_indices from vsk file... OK")

    def num_frames(self, ):
        return len(self.c3d_data)

    def set_frame(self, frame_index):
        self.frame_index = frame_index
        if frame_index < len(self.motions):
            self.robot.set_positions(self.motions[frame_index])

    def f(self, x):
        self.robot.set_positions(x)

        frame_data = self.c3d_data[self.frame_index]
        ret = 0.0
        w = 1.0

        for m, rhs in zip(self.robot.markers,
                          frame_data):
            if np.allclose((0.0, 0.0, 0.0), rhs):
                continue
            lhs = m.world_position()
            ret += w * np.linalg.norm(lhs - rhs) ** 2
        ret /= self.num_markers
        return ret

    def g(self, x):
        self.robot.set_positions(x)

        frame_data = self.c3d_data[self.frame_index]
        g = np.zeros(self.robot.ndofs)
        w = 1.0

        for m, rhs in zip(self.robot.markers,
                          frame_data):
            if np.allclose((0.0, 0.0, 0.0), rhs):
                continue
            lhs = m.world_position()
            body = m.bodynode
            offset = m.local_position()
            J = body.linear_jacobian(offset)
            g_i = 2.0 * w * (lhs - rhs).dot(J)
            g += g_i
        g /= self.num_markers
        DEBUG = False
        if DEBUG:
            import numdifftools as nd
            g_approx = nd.Gradient(self.f)
            lhs = g
            rhs = g_approx(x)
            print(lhs)
            print(rhs)
            dist = np.linalg.norm(lhs - rhs)
            print("dist = %.8f" % dist)
            print("OK" if np.allclose(lhs, rhs, atol=1e-4) else "NG!!!!")
        return g

    def solve(self, ):
        options = {'maxiter': 3000,
                   'ftol': 1e-12,
                   'disp': True,
                   'iprint': 2}
        tic = time()
        res = minimize(self.f,
                       jac=self.g,
                       x0=self.robot.positions(),
                       method="SLSQP",
                       options=options)
        toc = time()
        res['time'] = toc - tic
        self.motions.append(self.robot.positions())
        output_filename = str(self.c3d_filename)
        output_filename = output_filename.replace(".txt", ".motion.txt")
        output_filename = output_filename.replace(".c3d", ".motion.txt")
        self.save(output_filename)

        print(res)

        with open("summary.csv", "a+") as fout:
            args = (self.c3d_filename,
                    output_filename,
                    len(self.motions),
                    res['fun'])
            fout.write("%s,%s,%d,%.8f\n" % args)

    def step(self, ):
        self.solve()
        if self.frame_index + 1 < self.num_frames():
            self.set_frame(self.frame_index + 1)
        else:
            return False

    def save(self, filename):
        np.savetxt(filename, self.motions)
        print("save %d frames to %s" % (len(self.motions), filename))

    def render_with_ri(self, ri):
        ri.set_line_width(2.0)
        # ri.render_axes((0.0, 0.0, 0.0), 1.0)
        ri.set_line_width(1.0)

        if self.frame_index < len(self.c3d_data):
            frame_data = self.c3d_data[self.frame_index]
            for m, rhs in zip(self.robot.markers,
                              frame_data):
                lhs = m.world_position()
                ri.set_color(0.9, 0.5, 0.0)
                ri.render_sphere(lhs, 0.02)
                ri.set_color(0.0, 0.5, 0.9)
                ri.render_sphere(rhs, 0.02)
                ri.set_color(0.0, 1.0, 0.0)
                ri.render_line(lhs, rhs)

        for body in self.robot.bodynodes:
            ri.push()
            ri.mult_matrix(body.T)
            ri.set_line_width(2.0)
            # ri.render_axes((0.0, 0.0, 0.0), 0.3)
            alen = 0.1
            ri.set_color(1.0, 0.0, 0.0)
            ri.render_line((0.0, 0.0, 0.0), (alen, 0.0, 0.0))
            ri.set_color(0.0, 1.0, 0.0)
            ri.render_line((0.0, 0.0, 0.0), (0.0, alen, 0.0))
            ri.set_color(0.0, 0.0, 1.0)
            ri.render_line((0.0, 0.0, 0.0), (0.0, 0.0, alen))
            ri.set_line_width(1.0)
            ri.pop()

        # frame_data = self.c3d_data[self.frame_index]
        # for i, rhs in enumerate(frame_data):
        #     if np.linalg.norm(rhs) < 0.001:
        #         continue
        #     ri.set_color(0.0, 0.5, 0.9)
        #     ri.render_sphere(rhs, 0.02)


if __name__ == '__main__':
    import sys
    print('Example: process_mocap.py [.vsk] [.c3d]')
    _, vsk_filename, c3d_filename = sys.argv

    pydart.init()
    print('pydart initialization OK')
    world = MyWorld(vsk_filename, c3d_filename)

    win = pydart.gui.pyqt5.window.PyQt5Window(world)
    win.scene.set_camera(1)  # Z-up Camera
    win.run()

    # for i in range(500000):
    #     result = world.step()
    #     if result is not None and not result:
    #         break
