# This maintains the links for all sources used by this superbuild.
# Simply update this file to change the revision.
# One can use different revision on different platforms.
# e.g.
# if (UNIX)
#   ..
# else (APPLE)
#   ..
# endif()

superbuild_set_revision(pythonqt
  GIT_REPOSITORY http://github.com/commontk/PythonQt.git
  GIT_TAG patched-8)

set(PARAVIEW_VERSION 5.4)
superbuild_set_revision(paraview
  GIT_REPOSITORY https://gitlab.kitware.com/bjacquet/paraview.git
  GIT_TAG origin/5.4CustomForVeloview-withPointCloudRepPlugin)

superbuild_set_revision(veloview
    SOURCE_DIR ${CMAKE_SOURCE_DIR}/..
    DOWNLOAD_COMMAND "")

if (WIN32)
  superbuild_set_revision(pcap
    GIT_REPOSITORY http://github.com/patmarion/winpcap.git
    GIT_TAG master)
else()
  superbuild_set_revision(pcap
    URL "http://www.tcpdump.org/release/libpcap-1.4.0.tar.gz"
    URL_MD5 "56e88a5aabdd1e04414985ac24f7e76c")
endif()

# General
# another revision of boost is addedd inside Superbuild/common-superbuild/versions.txt
# but this file has a higher priority
superbuild_set_revision(boost
  URL "https://sourceforge.net/projects/boost/files/boost/1.63.0/boost_1_63_0.tar.gz"
  URL_MD5 7b493c08bc9557bbde7e29091f28b605)

superbuild_set_revision(eigen
  GIT_REPOSITORY https://github.com/eigenteam/eigen-git-mirror.git
  GIT_TAG 3.2.10)

superbuild_set_revision(liblas
  URL     "http://www.paraview.org/files/dependencies/libLAS-1.8.1.tar.bz2"
  URL_MD5 2e6a975dafdf57f59a385ccb87eb5919)
  
superbuild_set_revision(ceres
  GIT_REPOSITORY https://ceres-solver.googlesource.com/ceres-solver
  GIT_TAG 1.14.0)

superbuild_set_revision(glog
  GIT_REPOSITORY https://github.com/google/glog.git
  GIT_TAG 8d7a107d68c127f3f494bb7807b796c8c5a97a82)

superbuild_set_revision(pcl
  GIT_REPOSITORY git://github.com/PointCloudLibrary/pcl.git
  GIT_TAG pcl-1.8.1)

superbuild_set_revision(qhull
    GIT_REPOSITORY git://github.com/qhull/qhull.git
    GIT_TAG master)

superbuild_set_revision(flann
  GIT_REPOSITORY git://github.com/mariusmuja/flann.git
  GIT_TAG 1.9.1)
