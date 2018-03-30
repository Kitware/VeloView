add_external_project(veloview
  DEPENDS paraview qt pcap boost eigen liblas

  CMAKE_ARGS
    -DBUILD_SHARED_LIBS:BOOL=ON
    -DBUILD_TESTING:BOOL=OFF
    -DParaView_DIR:PATH=${SuperBuild_BINARY_DIR}/paraview/src/paraview-build
    -DEIGEN_INCLUDE_DIR:PATH=<INSTALL_DIR>/include/eigen3
    -DPYTHONQT_DIR:PATH=<INSTALL_DIR>
    -DVTK_DIR:PATH=${SuperBuild_BINARY_DIR}/paraview/src/paraview-build/VTK
    -DBOOST_ROOT:PATH=<INSTALL_DIR>
    -DBOOST_LIBRARYDIR:PATH=<INSTALL_DIR>/lib
    -DPCL_DIR:PATH=<INSTALL_DIR>/share/pcl-1.7/
)
