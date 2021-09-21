cmake_dependent_option(Boost_NO_BOOST_CMAKE "Boost_NO_BOOST_CMAKE" ON "NOT USE_SYSTEM_boost" OFF)

superbuild_add_project(lidarview
  DEPENDS paraview qt5 pcap boost eigen liblas yaml python3 pythonqt numpy
  DEPENDS_OPTIONAL pcl ceres opencv nanoflann g2o
  DEFAULT_ON
  CMAKE_ARGS
    #LidarView base configuration
    -DBUILD_SHARED_LIBS:BOOL=ON
    -DBUILD_TESTING:BOOL=OFF
    -DLV_BUILD_PLATFORM=${LV_BUILD_PLATFORM}
    -Dsuperbuild_python_version=${superbuild_python_version}
    -DParaView_DIR:PATH=${SuperBuild_BINARY_DIR}/common-superbuild/paraview/build
    -DCMAKE_CXX_STANDARD:STRING=${CMAKE_CXX_STANDARD}
    -Dparaview_version=${paraview_version}
    -Dqt_version:STRING=${qt_version}
    -DBoost_NO_BOOST_CMAKE:BOOL=${Boost_NO_BOOST_CMAKE}
    #$lidarview_appname dependencies options
    -DENABLE_pcl=${ENABLE_pcl}
    -DENABLE_ceres=${ENABLE_ceres}
    -DENABLE_opencv=${ENABLE_opencv}
    -DENABLE_nanoflann=${ENABLE_nanoflann}
    -DENABLE_g2o=${ENABLE_g2o}
    #$lidarview_appname features options
    -DLIDARVIEW_BUILD_SLAM=${LIDARVIEW_BUILD_SLAM}
    #$lidarview_appname additional configuration
)

# This Disable Boost autolinking feature and ease use of Boost as a dynamic library.
# Boost_USE_STATIC_LIBS is off by default, but sometimes that is not sufficient
# on windows (especially with MSVC ?)
if (WIN32 OR APPLE)
  superbuild_append_flags(cxx_flags "-DBOOST_ALL_NO_LIB" PROJECT_ONLY)
  superbuild_append_flags(cxx_flags "-DBOOST_ALL_DYN_LINK" PROJECT_ONLY)
endif()
