find_program(QT_QMAKE_EXECUTABLE NAMES qmake qmake4 qmake-qt4 qmake-mac PATHS
  $ENV{QTDIR}/bin
  DOC "The qmake executable for the Qt installation to use")
add_extra_cmake_args(
  -DQT_QMAKE_EXECUTABLE:FILEPATH=${QT_QMAKE_EXECUTABLE})
