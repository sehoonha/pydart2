Installation Guide
===================================

Install with pip (easy)
----------------------------------
In Ubuntu, PyDART2 can be easily installed using PyPI - the Python Package
Index.
The default Python environment is assumed to be Python3, although PyDART2 is also available in Python2.

The first step is to install DART 6.0.1 (You can skip this if you already have it!).
Please use your favorite method to install DART, such as, ..

.. code-block:: bash

   sudo apt-add-repository ppa:dartsim
   sudo apt-get update
   sudo apt-get install libdart6-all-dev


Please refer the official DART installation document
(<https://github.com/dartsim/dart/wiki/Installation>)
when you have problems.


The next step is to install SWIG, pip3, and PyQt4.

They can be installed by the following command:

.. code-block:: bash

   sudo apt-get install swig python3-pip python3-pyqt4 python3-pyqt4.qtopengl


The final step is to install PyDART2 using pip3.

.. code-block:: bash

   sudo pip3 install pydart2

All done! Please enjoy the simulation.

.. code-block:: bash

   $ python3
   >>> import pydart2 as pydart
   >>> pydart.init(verbose=True)
   Msg  [pydart2_api] Initialize pydart manager OK

For Python2 users, please apply the following commands:

.. code-block:: bash

   sudo apt-get install swig python-pip python-qt4 python-qt4-dev python-qt4-gl
   sudo pip install pydart2



Install from source code
----------------------------------
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


Install using CMake (Old-style)
----------------------------------
I also wrote CMakeLists.txt, which is an old-style cross compilation system
used in the original PyDART.
