cmake_dependent_option(Boost_NO_BOOST_CMAKE "Boost_NO_BOOST_CMAKE" ON "NOT USE_SYSTEM_boost" OFF)

superbuild_add_project(lidarview
  DEPENDS paraview qt5 pcap boost eigen liblas yaml python python3 pythonqt
  DEPENDS_OPTIONAL pcl ceres opencv nanoflann g2o
  DEFAULT_ON
  CMAKE_ARGS
    #LidarView base configuration
    -DBUILD_SHARED_LIBS:BOOL=ON
    -DBUILD_TESTING:BOOL=OFF
    -DLV_BUILD_ARCHITECTURE=${LV_BUILD_ARCHITECTURE}
    -DSOFTWARE_NAME=${SOFTWARE_NAME}
    -DSOFTWARE_VENDOR=${SOFTWARE_VENDOR}
    -DParaView_DIR:PATH=${SuperBuild_BINARY_DIR}/common-superbuild/paraview/build
    -DVTK_DIR:PATH=${SuperBuild_BINARY_DIR}/common-superbuild/paraview/build/VTK
    -DCMAKE_CXX_STANDARD:STRING=${CMAKE_CXX_STANDARD}
    -Dqt_version:STRING=${qt_version}
    -DPYTHONQT_DIR:PATH=<INSTALL_DIR>
    -DBoost_NO_BOOST_CMAKE:BOOL=${Boost_NO_BOOST_CMAKE}
    -DENABLE_pcl=${ENABLE_pcl}
    -DENABLE_ceres=${ENABLE_ceres}
    -DENABLE_opencv=${ENABLE_opencv}
    -DENABLE_nanoflann=${ENABLE_nanoflann}
    -DENABLE_g2o=${ENABLE_g2o}
    -DENABLE_slam=${ENABLE_slam}
    -DENABLE_old_slam=${ENABLE_old_slam}
    #$lidarview_appname additional configuration
)

if (WIN32 OR APPLE)
  # These options are useful to use Boost as a dynamic library.
  # Boost_USE_STATIC_LIBS is off by default, but sometimes that is not sufficient
  # on windows (especially with MSVC ?)
  superbuild_append_flags(cxx_flags "-DBOOST_ALL_NO_LIB" PROJECT_ONLY)
  superbuild_append_flags(cxx_flags "-DBOOST_ALL_DYN" PROJECT_ONLY)
endif()
