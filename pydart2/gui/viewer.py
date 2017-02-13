from pydart2.gui.pyqt4.window import PyQT4Window
from pydart2.gui.glut.window import GLUTWindow


def launch_window(sim, win, default_camera):
    if default_camera is not None:
        win.scene.set_camera(default_camera)
    win.run()


def launch_pyqt4(sim, title=None, default_camera=None):
    win = PyQT4Window(sim, title)
    launch_window(sim, win, default_camera)


def launch_glut(sim, title=None, default_camera=None):
    win = GLUTWindow(sim, title)
    launch_window(sim, win, default_camera)


def launch(sim, title=None, default_camera=None):
    """
    default is PyQt4
    """
    launch_pyqt4(sim, title, default_camera)
    # launch_glut(sim, title, default_camera)
