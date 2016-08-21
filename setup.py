from setuptools import setup
from setuptools import find_packages
from distutils.core import Extension
from sys import platform as _platform


DIR = 'pydart2/'

CXX_FLAGS = '-Wall -msse2 -fPIC -std=c++11 -Xlinker -rpath /usr/local/lib '
CXX_FLAGS += '-O3 -DNDEBUG -shared '
CXX_FLAGS += '-g -fno-omit-frame-pointer -fno-inline-functions '
CXX_FLAGS += '-fno-inline-functions-called-once -fno-optimize-sibling-calls '

include_dirs = list()
include_dirs += ['/usr/include']
include_dirs += ['/usr/include/eigen3']
include_dirs += ['/usr/include/python2.7']
include_dirs += ['/usr/include/bullet']
include_dirs += ['/usr/local/include']
include_dirs += ['/usr/local/include/eigen3']
include_dirs += ['/usr/local/include/python2.7']
include_dirs += ['/usr/local/include/bullet']
include_dirs += ['/usr/local/lib/python2.7/dist-packages/numpy/core/include/']
include_dirs += ['/usr/lib/python2.7/dist-packages/numpy/core/include/']

libraries = list()
libraries += ['dart', 'dart-gui']
libraries += ['dart-optimizer-ipopt', 'dart-optimizer-nlopt',
              'dart-planning', 'dart-utils', 'dart-utils-urdf']
libraries += ['python2.7']
if _platform == "linux" or _platform == "linux2":
    libraries += ['GL', 'glut', 'Xmu', 'Xi']
elif _platform == "darwin":
    libraries += ['GLUT', 'Cocoa', 'OpenGL']
libraries += ['BulletDynamics', 'BulletCollision',
              'LinearMath', 'BulletSoftBody']


pydart2_api = Extension('_pydart2_api',
                        define_macros=[('MAJOR_VERSION', '1'),
                                       ('MINOR_VERSION', '0')],
                        include_dirs=include_dirs,
                        libraries=libraries,
                        library_dirs=['/usr/local/lib'],
                        extra_compile_args=CXX_FLAGS.split(),
                        swig_opts=['-c++'],
                        sources=[DIR + 'pydart2_api.cpp',
                                 DIR + 'pydart2_draw.cpp',
                                 DIR + 'pydart2_api.i'],
                        depends=[DIR + 'pydart2_api.h',
                                 DIR + 'pydart2_draw.h',
                                 DIR + 'numpy.i'])


setup(name='pydart2',
      version='0.3.13',
      description='Python Interface for DART Simulator',
      url='https://github.com/sehoonha/pydart2',
      author='Sehoon Ha',
      author_email='sehoon.ha@gmail.com',
      license='BSD',
      install_requires=[
          'numpy', 'PyOpenGL', 'PyOpenGL_accelerate'
      ],
      keywords=['physics', 'robotics', 'simulation'],
      classifiers=['Development Status :: 2 - Pre-Alpha',
                   'License :: OSI Approved :: BSD License',
                   'Operating System :: POSIX :: Linux',
                   'Programming Language :: Python :: 2 :: Only',
                   'Topic :: Games/Entertainment :: Simulation'],
      packages=find_packages(),
      ext_modules=[pydart2_api])
