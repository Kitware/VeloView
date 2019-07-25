superbuild_add_project(lidarview
  DEPENDS paraview qt5 pcap boost eigen liblas yaml
  DEFAULT_ON
  CMAKE_ARGS
    -DBUILD_SHARED_LIBS:BOOL=ON
    -DBUILD_TESTING:BOOL=OFF
    -DParaView_DIR:PATH=${SuperBuild_BINARY_DIR}/common-superbuild/paraview/build
    -DEIGEN_INCLUDE_DIR:PATH=<INSTALL_DIR>/include/eigen3
    -DPYTHONQT_DIR:PATH=<INSTALL_DIR>
    -DVTK_DIR:PATH=${SuperBuild_BINARY_DIR}/common-superbuild/paraview/build/VTK
    -DCMAKE_CXX_STANDARD:STRING=${CMAKE_CXX_STANDARD}
    -DBOOST_ROOT:PATH=<INSTALL_DIR>
    -DBOOST_LIBRARYDIR:PATH=<INSTALL_DIR>/lib
    -Dqt_version:STRING=${qt_version}
    -DPCL_DIR:PATH=<INSTALL_DIR>/share/pcl-1.8/
    -DYAML_DIR:PATH=<INSTALL_DIR>/include/yaml-cpp/
    -DENABLE_pcl=${ENABLE_pcl}
    -DENABLE_ceres=${ENABLE_ceres}
    -DENABLE_opencv=${ENABLE_opencv}
    -DENABLE_nanoflann=${ENABLE_nanoflann}
)

if (WIN32 OR APPLE)
  # These options are useful to use Boost as a dynamic library.
  # Boost_USE_STATIC_LIBS is off by default, but sometimes that is not sufficient
  # on windows (especially with MSVC ?)
  superbuild_append_flags(cxx_flags "-DBOOST_ALL_NO_LIB" PROJECT_ONLY)
  superbuild_append_flags(cxx_flags "-DBOOST_ALL_DYN" PROJECT_ONLY)
endif()
