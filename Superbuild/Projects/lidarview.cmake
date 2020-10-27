superbuild_add_project(lidarview
  DEPENDS paraview qt5 pcap boost eigen liblas yaml python python3 pythonqt
  DEPENDS_OPTIONAL pcl ceres opencv nanoflann g2o
  DEFAULT_ON
  CMAKE_ARGS
    -DBUILD_SHARED_LIBS:BOOL=ON
    -DBUILD_TESTING:BOOL=OFF
    -DParaView_DIR:PATH=${SuperBuild_BINARY_DIR}/common-superbuild/paraview/build
    -DVTK_DIR:PATH=${SuperBuild_BINARY_DIR}/common-superbuild/paraview/build/VTK
    -Dqt_version:STRING=${qt_version}
    -DPYTHONQT_DIR:PATH=<INSTALL_DIR>
    -DBOOST_ROOT:PATH=<INSTALL_DIR>
    -DBOOST_LIBRARYDIR:PATH=<INSTALL_DIR>/lib
    -DYAML_DIR:PATH=<INSTALL_DIR>/include/yaml-cpp
    -DEIGEN_INCLUDE_DIR:PATH=<INSTALL_DIR>/include/eigen3
    -DEIGEN3_DIR:PATH=<INSTALL_DIR>/share/eigen3/cmake
    -DPCL_DIR:PATH=<INSTALL_DIR>/share/pcl-1.10
    -DENABLE_pcl=${ENABLE_pcl}
    -DENABLE_ceres=${ENABLE_ceres}
    -DENABLE_opencv=${ENABLE_opencv}
    -DENABLE_nanoflann=${ENABLE_nanoflann}
    -DENABLE_g2o=${ENABLE_g2o}
    -DENABLE_slam=${ENABLE_slam}
    -DENABLE_old_slam=${ENABLE_old_slam}
)

if (WIN32 OR APPLE)
  # These options are useful to use Boost as a dynamic library.
  # Boost_USE_STATIC_LIBS is off by default, but sometimes that is not sufficient
  # on windows (especially with MSVC ?)
  superbuild_append_flags(cxx_flags "-DBOOST_ALL_NO_LIB" PROJECT_ONLY)
  superbuild_append_flags(cxx_flags "-DBOOST_ALL_DYN" PROJECT_ONLY)
endif()

# reset boost RPATH on OSX
# this is applied on LidarView app and tests
# for more information see comments in lidarview.osx-boost-rpath.cmake
if (APPLE)
  superbuild_project_add_step(osx-boost-rpath
    COMMAND   "${CMAKE_COMMAND}"
              -Dinstall_location:PATH=<INSTALL_DIR> #location to get LidarView
              -P "<SOURCE_DIR>/Superbuild/lidarview-superbuild/Projects/scripts/lidarview.osx-boost-rpath.cmake"
    DEPENDEES install
    COMMENT   "Reset rpath for all boost dependencies"
    WORKING_DIRECTORY <BINARY_DIR>)
endif ()
