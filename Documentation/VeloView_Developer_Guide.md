# Table of contents
1. [VeloView dependencies](#dependencies)
    1. [PCAP library](#pcap-library)
    2. [Boost library](#boost-library)
    3. [Qt library](#qt-library)
    4. [Python](#python)
    5. [Python Qt library](#python-qt-library)
    6. [Paraview and VTK](#paraview-vtk)
2. [Configure and build instructions](#configure-build)
    1. [Superbuild Overview](#superbuild-overview)
    2. [Windows dependencies](#windows-dependencies)
    3. [Windows build instructions](#windows-build-instructions)
    2. [Linux dependencies](#linux-dependencies)
    3. [Linux build instructions](#linux-build-instructions)

## VeloView dependencies <a name="dependencies"></a>
The VeloView application and libraries have several external library dependencies. As explained in [Superbuild Overview](#superbuild-overview), **most of the dependencies will be downloaded and compiled automatically** during the build step. See [Configure and build instructions](#configure-build).

### PCAP library <a name="pcap-library"></a>
The required pcap version is 1.4.
Pcap is required to support saving captured packets to a file, and reading files containing saved packets.
On Mac/Linux, only libpcap is required.
On Windows, we use the Winpcap project which includes libpcap but also includes Windows specific drivers. Since the winpcap project only provides Visual Studio project files, which may be out dated, the superbuild does not attempt to compile winpcap. Instead, a git repository containing headers and precompiled .lib and .dll files is used. The repository url is https://github.com/patmarion/winpcap.

### Boost library <a name="boost-library"></a>
The required boost version is 1.63.
Boost is used for threading and synchronization, network communication and handling of the filesystem.

### Qt library <a name="qt-library"></a>
The required Qt version is 5.10.
Qt is a desktop widget library that is used to provide user interface elements like windows and menus across the supported platforms Windows, Mac, and Linux.

### Python <a name="python"></a>
The required Python version is 2.7.
VeloView uses libpython to embed a Python interpreter in the VeloView application.
The core VeloView features are implemented in C++ libraries, and the libraries are wrapped for Python using VTK's Python wrapping tools.

### PythonQt <a name="python-qt-library"></a>
PythonQt version is "patch_8" (see Superbuild/version.txt).
PythonQt is used to build Qt applications using Python.
PythonQt has support for wrapping types derived from Qt objects and VTK objects.

### Paraview (and VTK) <a name="paraview-vtk"></a>
The required ParaView version is 5.4.
The required VTK version is 8.1.
The ParaView repository includes VTK, so the superbuild only needs to checkout and build ParaView in order to satisfy both dependencies.
A specific git commit sha1 is used instead of a specific released version.
The commit sha1 is very similar to the version 4.0 release but it has a few commits from the ParaView master branch cherry-picked onto it.
The commits added are those that resolve some issues with the Python console and add the PythonQtPlugin for ParaView.
The PythonQtPlugin is a small plugin that initializes the PythonQt library and makes it available in the ParaView Python console.

## Configure and build instructions <a name="configure-build"></a>
The VeloView software is hosted in git repositories that live on github.com (public version) and gitlab.kitware.com (internal version).

### Superbuild Overview <a name="superbuild-overview"></a>
VeloView can use a cmake *superbuild* to download and compile third party projects that are dependencies of VeloView.
The superbuild is not mandatory but it is recommended. It eases building a lot for new developers.
The superbuild will give you the option to use system installations of third party projects instead of compiling them as a superbuild step.
Some dependencies, on certain platforms, must be compiled by the superbuild, and for them there is no option to use a system version.

### Windows dependencies <a name="windows-dependencies"></a>
- cmake version 3.7.2 is confirmed to work (lower versions may not work, higher versions will work), cmake is available at <https://cmake.org/>
- ninja version 1.8.2 or higher, available at <https://github.com/ninja-build/ninja/releases>. There is no installer for this tool. You must extract the binary ninja.exe from ninja-win.zip and place it inside a directory that is inside your %PATH% environnement variable, such as C:\\Windows
- Microsoft Visual Studio **14** (2015) **Express** ("Desktop"). You can use this link to download the installer: <http://go.microsoft.com/fwlink/?LinkId=615464> This installer is pretty simple (no special options).
- git: we recommand using "Git for Windows" available at <https://gitforwindows.org/>
- Qt 5.10.0 *(this dependency will be built automatically in the future)*. You can download the installer here: <https://download.qt.io/official_releases/qt/5.10/5.10.0/qt-opensource-windows-x86-5.10.0.exe>. When installing you can keep the suggested installation path. Here is a walkthrough of the installer:  click "Next" > "Skip" > "Next" > keep default install path (advised) and click "Next" > Unfold "Qt" then unfold "Qt 5.10.0" and tick "**MSVC 2015 64-bits**" then click "Next" > "Next" > "Install" > wait for it to install then click "Next" > untick "Launch Qt Creator" and click "Finish"

### Windows build instructions <a name="windows-build-instructions"></a>
1. clone VeloView's source code repository to a directory of your chosing, for example:

    `cd <work-directory>`

    `git clone <git url to VeloView repository> VeloView-source`

    * you will have to know the path to this directory for step 6.
    * moving this directoy in the future will break all build environnements that were using it (you will have to redo steps 6. and 7.)


2. clone the submodule that is used to provide the Superbuild:

    `cd VeloView-source`

    `git submodule update --init Superbuild/common-superbuild`

    * this command must be executed inside the previously cloned repository

3. create a new directory to store the build.

    `mkdir C:\VeloView-build`

    * You can use the Windows file explorer to create this directory
    * **This directory must not be inside the VeloView source code directory**
    * **the path to this directory must be short** because Windows has limitations on the maximum length of file paths. We suggest that you use a directory at the root of a drive, like *C:\\VeloView-build*


4. open the appropriate command prompt:

    `Windows Start Menu > Visual Studio 2015 > "VS2015 x86 x64 Cross Tools Command Prompt"`

    * tip: for the next steps it is possible to copy a command and then past it inside the prompt with shift+insert or right-click
    * this command prompt has some environnement variables set correctly to allow building (compiler path, etc)
    * alternatively it is possible to use a standard cmd.exe windows prompt (in which Administrative priviledges should not be enabled for security) by entering the command: `"C:\Program Files (x86)\Microsoft Visual Studio 14.0\VC\vcvarsall.bat" x86_amd64`


5. inside the command prompt, go to the build directory you created in step 3 by entering the command

    `cd /d "C:\VeloView-build"`

    * Adapt the path to your own build directory created in step 3.
    * `/d` is allows to `cd` to a directory that is not on the same drive as your current path


6. inside the command prompt configure the build by entering:

    `cmake <work-directory>\Veloview-source\Superbuild -GNinja -DCMAKE_BUILD_TYPE=Release -DUSE_SYSTEM_qt5=True -DQt5_DIR="C:/Qt/Qt5.10.0/5.10.0/msvc2015_64/lib/cmake/Qt5"`

    * Take note that this command mentions the subdirectory "Superbuild" inside the source directory and not the source directory itself.
    * Take note that the Qt5_DIR path must use **forward slashes** (like if it was an Unix PATH), because MSVC would otherwise take "\\Q" as a build option.
    * If you changed the default Qt installation path, you will have to adapt this command.
    * You can use absolute or relative path to point to the VeloView source directory.
    * Should you want to build in RelWithDebInfo mode (in order to attach a debugger for instance), replace "Release" by "RelWithDebInfo".
    * This command should show no errors, else they must be fixed.


8. inside the command prompt, start building by entering:

    `ninja`

    * Building from scratch can take from 45 minutes to 3 hours depending on your hardware.
    * By default ninja will use all cores on your machine, but you can restrict the number of cores used by using `ninja -jN` (replace N by the number of cores to use).


9. if you modified only VeloView and want to rebuild incrementally (incrementaly = only modified files are rebuilded), enter the commands:

    `cd common-superbuild/veloview/build`

    `ninja install`

    * Incremental builds are much faster than the first build.
    * it is also possible to run `ninja` from the top of the build directory like you did the first time, but that will take longer because all dependencies are checked for changes


### Linux dependencies <a name="linux-dependencies"></a>
The following packages are needed to build on Ubuntu 16.04:

- build-essential
- cmake
- git
- flex
- byacc
- python-minimal
- python2.7-dev
- libxext-dev
- libxt-dev
- libbz2-dev
- zlib1g-dev
- freeglut3-dev
- pkg-config


### Linux build instructions <a name="linux-build-instructions"></a>
1. clone VeloView's source code repository to a directory of your chosing, for example:

    `cd <work-directory>`

    `git clone <git url to VeloView repository> VeloView-source`

    * moving this directoy in the future will break all build environnements that were using it (you will have to redo steps 6. and 7.)


2. clone the submodule that is used to provide the Superbuild:

    `cd VeloView-source`

    `git submodule update --init Superbuild/common-superbuild`

    * this command must be executed inside the previously cloned repository


3. create a new directory to store the build.

    `mkdir <work-directory>/VeloView-build`

    * **This directory must not be inside the VeloView source code directory**


4. configure the build by entering:

    `cd <work-directory>/VeloView-build`

    `cmake <work-directory>/VeloView-source/Superbuild -DCMAKE_BUILD_TYPE=Release`

    * by default the generator used is **make**, if you prefer to use **ninja**, add the option `-GNinja`
    * Take note that this command mentions the subdirectory "Superbuild" inside the source directory and not the source directory itself.


5. start building by entering:

    `make -j<N>`

   * replace `<N>` by the number of cores you want to use


9. if you modified only VeloView and want to rebuild incrementally (incrementaly = only modified files are rebuilded), enter the commands:

    `cd common-superbuild/veloview/build`

    `make install`

    * Incremental builds are much faster than the first build.
    * it is also possible to run `make` from the top of the build directory like you did the first time, but that will take longer because all dependencies are checked for changes
