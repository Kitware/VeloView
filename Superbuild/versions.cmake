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
  if (64bit_build)
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
  URL "http://download.qt-project.org/archive/qt/4.8/4.8.2/qt-everywhere-opensource-src-4.8.2.tar.gz"
  URL_MD5 3c1146ddf56247e16782f96910a8423b)

add_revision(pythonqt
  GIT_REPOSITORY git://github.com/commontk/PythonQt.git
  GIT_TAG patched-3)

set(PARAVIEW_VERSION 4.3)
add_revision(paraview
  GIT_REPOSITORY git://github.com/gcasey/ParaView.git
  GIT_TAG origin/veloview)

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
  URL "http://downloads.sourceforge.net/project/boost/boost/1.50.0/boost_1_50_0.tar.gz"
  URL_MD5 dbc07ab0254df3dda6300fd737b3f264)

add_revision(eigen
  URL http://vtk.org/files/support/eigen-3.1.2.tar.gz
  URL_MD5 bb639388192cb80f1ee797f5dbdbe74f)

#add_revision(liblas
#  GIT_REPOSITORY git://github.com/libLAS/libLAS
#  GIT_TAG 6e8657336ba445fcec3c9e70c2ebcd2e25af40b9)
add_revision(liblas
  GIT_REPOSITORY git://github.com/mwoehlke-kitware/libLAS.git
  GIT_TAG fix-windows-stdint)
