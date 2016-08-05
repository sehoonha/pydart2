Installation Guide
===================================

Easy install with pip
============================
In Ubuntu, PyDART2 can be easily installed using PyPI - the Python Package
Index.

The first step is to install DART 6.0.1.
For example, you can use apt-get to install DART.

.. code-block:: bash

   > sudo apt-add-repository ppa:dartsim
   > sudo apt-get update
   > sudo apt-get install libdart6-all-dev


Please refer the official DART installation document
(<https://github.com/dartsim/dart/wiki/Installation>)
when you have problems.


The next step is to install SWIG, PIP, and PyQt4.
They can be installed by the following command.

.. code-block:: bash

   sudo apt-get install swig python-pip python-qt4 python-qt4-dev python-qt4-gl

The final step is to install PyDART2 using pip.

.. code-block:: bash

   sudo pip install pydart2   

All done! Please enjoy the simulation.

Install from source code using setup.py
=======================================
Sometimes, you want to edit source codes by yourself.
For the following steps, I assumed that you already installed the required
packages - swig, pip, PyQt4, and so on.

First, please check out the repository.

.. code-block:: bash

   git clone https://github.com/sehoonha/pydart2.git
   cd pydart

The next step is to compile the package using setup.py

.. code-block:: bash

    python setup.py build build_ext

The final step is to install the python package as a development.

.. code-block:: bash

    python setup.py develop


Old-style install using CMake
=======================================
I also wrote CMakeLists.txt, which is an old-style cross compilation system
used in the original PyDART.
