add_external_project(veloview
  DEPENDS paraview qt pcap boost

  CMAKE_ARGS
    -DBUILD_SHARED_LIBS:BOOL=ON
    -DParaView_DIR:PATH=${SuperBuild_BINARY_DIR}/paraview/src/paraview-build
    -DPYTHONQT_DIR:PATH=<INSTALL_DIR>
)
