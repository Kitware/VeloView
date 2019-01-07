# This maintains the links for all sources used by this superbuild.
# Simply update this file to change the revision.
# One can use different revision on different platforms.
# e.g.
# if (UNIX)
#   ..
# else (APPLE)
#   ..
# endif()

if (WIN32)
  if (VV_BUILD_ARCHITECTURE EQUAL 64)
    add_revision(python
      URL "http://www.paraview.org/files/dependencies/python+deps.tar.bz2"
      URL_MD5 "4318b8f771eda5606d9ce7f0be9f82e1")
  else ()
    add_revision(python
      URL "http://www.paraview.org/files/dependencies/python+deps-x32.tar.bz2"
      URL_MD5 "6ba441784a672e08379d23ddd61146f0")
  endif ()
elseif (CROSS_BUILD_STAGE STREQUAL "CROSS")
  add_revision(python
    URL "http://www.paraview.org/files/dependencies/Python-2.7.3_CMake-7d1eb56.tar.bz2"
    URL_MD5 "48121a265837f825b1136ca8cf9bc4cd")
else()
  add_revision(python
    URL "http://paraview.org/files/dependencies/Python-2.7.11.tgz"
    URL_MD5 "6b6076ec9e93f05dd63e47eb9c15728b")
endif()

add_revision(qt
  URL "http://download.qt-project.org/archive/qt/4.8/4.8.6/qt-everywhere-opensource-src-4.8.6.tar.gz"
  URL_MD5 2edbe4d6c2eff33ef91732602f3518eb)

add_revision(pythonqt
  GIT_REPOSITORY git://github.com/commontk/PythonQt.git
  GIT_TAG patched-6)

set(PARAVIEW_VERSION 5.1)
add_revision(paraview
  GIT_REPOSITORY https://gitlab.kitware.com/bjacquet/paraview.git
  GIT_TAG origin/point-cloud-rep)

add_revision(veloview
    SOURCE_DIR ${CMAKE_SOURCE_DIR}/..
    DOWNLOAD_COMMAND "")

if (WIN32)
  add_revision(pcap
    GIT_REPOSITORY git://github.com/patmarion/winpcap.git
    GIT_TAG master)
else()
  add_revision(pcap
    URL "http://www.tcpdump.org/release/libpcap-1.4.0.tar.gz"
    URL_MD5 "56e88a5aabdd1e04414985ac24f7e76c")
endif()

add_revision(boost
  URL "https://sourceforge.net/projects/boost/files/boost/1.63.0/boost_1_63_0.tar.gz"
  URL_MD5 7b493c08bc9557bbde7e29091f28b605)

add_revision(eigen
  GIT_REPOSITORY https://github.com/eigenteam/eigen-git-mirror.git
  GIT_TAG 3.2.0)

#add_revision(liblas
#  GIT_REPOSITORY git://github.com/libLAS/libLAS
#  GIT_TAG 6e8657336ba445fcec3c9e70c2ebcd2e25af40b9)
add_revision(liblas
  GIT_REPOSITORY git://github.com/bastienjacquet/libLAS.git
  GIT_TAG fix-windows-stdint)

add_revision(pcl
  GIT_REPOSITORY git://github.com/PointCloudLibrary/pcl.git
  GIT_TAG pcl-1.8.1)

add_revision(qhull
    GIT_REPOSITORY git://github.com/qhull/qhull.git
    GIT_TAG master)

add_revision(flann
  GIT_REPOSITORY git://github.com/gcasey/flann.git
  GIT_TAG 6f6eabaf4e3741f556a12255a0d750b35bf166c0)

add_revision(zlib
  URL "http://www.paraview.org/files/dependencies/zlib-1.2.7.tar.gz"
  URL_MD5 60df6a37c56e7c1366cca812414f7b85)
# NOTE: if updating zlib version, fix patch in zlib.cmake

add_revision(png
  URL "http://paraview.org/files/dependencies/libpng-1.4.8.tar.gz"
  URL_MD5 49c6e05be5fa88ed815945d7ca7d4aa9)