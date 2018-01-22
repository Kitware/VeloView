superbuild_add_project(veloview
  DEPENDS paraview qt pcap boost eigen liblas
  DEFAULT_ON
  CMAKE_ARGS
    -DBUILD_SHARED_LIBS:BOOL=ON
    -DBUILD_TESTING:BOOL=OFF
    -DParaView_DIR:PATH=${SuperBuild_BINARY_DIR}/common-superbuild/paraview/build
    -DEIGEN_INCLUDE_DIR:PATH=<INSTALL_DIR>/include/eigen3
    -DPYTHONQT_DIR:PATH=<INSTALL_DIR>
    -DVTK_DIR:PATH=${SuperBuild_BINARY_DIR}/common-superbuild/paraview/build/VTK
    -DCMAKE_CXX_STANDARD:STRING=${CMAKE_CXX_STANDARD}
    -Dqt_version:STRING=${qt_version}
)
