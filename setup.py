from setuptools import setup
from distutils.core import Extension


DIR = 'pydart2/'

CXX_FLAGS = '-Wall -msse2 -fPIC -std=c++11 -Xlinker -rpath /usr/local/lib '
CXX_FLAGS += '-O3 -DNDEBUG -shared '
CXX_FLAGS += '-g -fno-omit-frame-pointer -fno-inline-functions '
CXX_FLAGS += '-fno-inline-functions-called-once -fno-optimize-sibling-calls '

libraries = list()
libraries += ['python2.7']
# libraries += ['dart', 'dart-core']
# libraries += ['GL', 'glut', 'Xmu', 'Xi']
# libraries += ['BulletDynamics', 'BulletCollision',
#               'LinearMath', 'BulletSoftBody']


pydart2_api = Extension('_pydart2_api',
                        define_macros=[('MAJOR_VERSION', '1'),
                                       ('MINOR_VERSION', '0')],
                        include_dirs=['/usr/local/include',
                                      '/usr/include/eigen3',
                                      '/usr/include/python2.7',
                                      '/usr/include/bullet' ],
                        libraries=libraries,
                        library_dirs=['/usr/local/lib'],
                        extra_compile_args=CXX_FLAGS.split(),
                        swig_opts=['-c++'],
                        sources=[DIR + 'pydart2_api.cpp',
                                 DIR + 'pydart2_api.i'],
                        depends=[DIR + 'pydart2_api.h', ])


setup(name='pydart2',
      version='0.3.3',
      description='Python Interface for DART Simulator',
      url='https://github.com/sehoonha/pydart2',
      author='Sehoon Ha',
      author_email='sehoon.ha@gmail.com',
      license='BSD',
      install_requires=[
          'numpy',
      ],
      packages=['pydart2'],
      ext_modules=[pydart2_api])
