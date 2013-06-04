========================
VeloView Developer Guide
========================

---------------------------------------------------------------------------
Software developer documentation for the VeloView application and libraries
---------------------------------------------------------------------------

:copyright: Copyright (c) 2013, Velodyne Lidar. All rights reserved.
:version: 1.0.5

.. contents:: Table of Contents
.. section-numbering::
.. target-notes::


Build instructions
==================


Superbuild overview
-------------------


Getting the source code
-----------------------

You'll use git to checkout the VeloView source code repository.  The software
is hosted on GitHub.  The checkout command is::

    git clone git://github.com/patmarion/veloview-superbuild.git


External dependencies
---------------------

The VeloView application and libraries have several external library dependencies.
As explained in the `Superbuild overview`_, the dependencies will be downloaded
and compiled automaticaly during the build step.  See `Configure and build instructions`_.

pcap
~~~~

Boost
~~~~~

Qt
~~

Python
~~~~~~

PythonQt
~~~~~~~~

VTK and ParaView
~~~~~~~~~~~~~~~~


Configure and build instructions
--------------------------------

The build requires cmake::

    mkdir build
    cd build
    cmake ../veloview-superbuild


Packaging
---------


Packaging for MacOSX
~~~~~~~~~~~~~~~~~~~~


Packaging for Windows
~~~~~~~~~~~~~~~~~~~~~


Developer Guide
===============

Source code organization
------------------------


ParaView plugin
---------------


VTK readers
-----------


VeloView application
--------------------


Python implementation
---------------------


PythonQt
--------


