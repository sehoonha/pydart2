from setuptools import setup
from setuptools import find_packages


setup(name='pydart2',
      version='0.3.10',
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
      packages=find_packages())

