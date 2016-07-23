from setuptools import setup
# from setuptools.command.install import install
from setuptools.command.build_py import build_py


class Pydart2Build(build_py):
    def run(self):
        print("[pydart2] pre-build")
        build_py.run(self)
        print("[pydart2] post-build")


setup(name='pydart2',
      version='0.2',
      description='Python Interface for DART Simulator',
      url='https://github.com/sehoonha/pydart2',
      author='Sehoon Ha',
      author_email='sehoon.ha@gmail.com',
      license='BSD',
      cmdclass={'build_py': Pydart2Build},
      packages=['pydart2'],
      zip_safe=False)
