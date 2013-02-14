# This maintains the links for all sources used by this superbuild.
# Simply update this file to change the revision.
# One can use different revision on different platforms.
# e.g.
# if (UNIX)
#   ..
# else (APPLE)
#   ..
# endif()

add_revision(qt
  URL "http://releases.qt-project.org/qt4/source/qt-everywhere-opensource-src-4.8.2.tar.gz"
  URL_MD5 3c1146ddf56247e16782f96910a8423b)

add_revision(paraview
  URL "http://paraview.org/files/v3.98/ParaView-3.98.1-source.tar.gz"
  URL_MD5 e91992b825be4c8e5ef8639fdfa0b557)

add_revision(velodyneviewer
  GIT_REPOSITORY git://kwsource.kitwarein.com/miscellaneousprojectsuda/velodyneviewer.git
  GIT_TAG master)

add_revision(pcap
  URL "http://www.tcpdump.org/release/libpcap-1.3.0.tar.gz"
  URL_MD5 "f78455a92622b7a3c05c58b6ad1cec7e")
