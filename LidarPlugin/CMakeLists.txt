project(LidarPlugin)
cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

include_directories("ctk")

#-----------------------------------------------------------------------------
# Temporary Fix
#-----------------------------------------------------------------------------

message(AUTHOR_WARNING "Until the 'Speedup interpolator' merge request is integrated to VTK, we provide the patch")
set(interpolator_pach_until_vtk_update
  vtkPatch/vtkCustomPiecewiseFunction.cxx
  vtkPatch/vtkCustomQuaternionInterpolator.cxx
  vtkPatch/vtkCustomTupleInterpolator.cxx
)

#-----------------------------------------------------------------------------
# Option to hide/show external warning library
#-----------------------------------------------------------------------------

set(SYSTEM_OPTION)
option(BUILD_HIDE_WARNING_FROM_EXTERNAL_DEPENDENCIES "hide external libraries warning, by handling them as system libraries" ON)
if(BUILD_HIDE_WARNING_FROM_EXTERNAL_DEPENDENCIES)
  set(SYSTEM_OPTION SYSTEM)
endif()

#-----------------------------------------------------------------------------
# Handle plugin library dependencies
#-----------------------------------------------------------------------------

#--------------------------------------
# ParaView dependency
#--------------------------------------
if(ParaView_SOURCE_DIR)

  include_directories(${SYSTEM_OPTION} ${VTK_INCLUDE_DIRS})
  if(PARAVIEW_BUILD_QT_GUI)
    include(${QT_USE_FILE})
    include_directories(${SYSTEM_OPTION} ${PARAVIEW_GUI_INCLUDE_DIRS})
  endif(PARAVIEW_BUILD_QT_GUI)

else(ParaView_SOURCE_DIR)
  find_package(ParaView REQUIRED)
  include(${PARAVIEW_USE_FILE})

endif(ParaView_SOURCE_DIR)

vtk_module_load(vtklibproj4)
include_directories(${SYSTEM_OPTION} ${vtklibproj4_INCLUDE_DIRS})
include_directories(${SYSTEM_OPTION} ${PYTHONQT_INCLUDE_DIRS})

#--------------------------------------
# pcap dependency
#--------------------------------------
find_library(PCAP_LIBRARY pcap DOC "pcap library")
find_path(PCAP_INCLUDE_DIR pcap.h DOC "pcap include directory")
mark_as_advanced(PCAP_LIBRARY PCAP_INCLUDE_DIR)
include_directories(${SYSTEM_OPTION} ${PCAP_INCLUDE_DIR})

#--------------------------------------
# liblas dependency
#--------------------------------------
set(LASNAME las)
if(WIN32)
  set(LASNAME liblas)
endif()

find_library(liblas_LIBRARY ${LASNAME} DOC "liblas library")
find_path(liblas_INCLUDE_DIR liblas/version.hpp DOC "liblas include directory")
mark_as_advanced(liblas_LIBRARY liblas_INCLUDE_DIR)
include_directories(${SYSTEM_OPTION} ${liblas_INCLUDE_DIR})

#--------------------------------------
# Eigen dependency
#--------------------------------------
find_package(Eigen REQUIRED)
include_directories(${SYSTEM_OPTION} ${EIGEN_INCLUDE_DIR})

#--------------------------------------
# Yaml dependency
#--------------------------------------
find_package(yaml-cpp REQUIRED)
include_directories(${SYSTEM_OPTION} ${YAML_CPP_INCLUDE_DIR})

#--------------------------------------
# PCL dependency
#--------------------------------------
option(ENABLE_pcl "PCL will be required for filters using pcl (PCLRansacFilter, Slam, ...)" OFF)
if (ENABLE_pcl)
  find_package(PCL REQUIRED)
  include_directories(${SYSTEM_OPTION} ${PCL_INCLUDE_DIRS})
  add_definitions(${PCL_DEFINITIONS})
endif(ENABLE_pcl)

#--------------------------------------
# Boost dependency
#--------------------------------------
set(Boost_USE_MULTITHREADED ON)
# the expected behavior of find_package when  used wiht REQUIRED and OPTIONAL_COMPONENTS
# it not the one expected, as it also required the optional component...
find_package(Boost REQUIRED COMPONENTS system thread chrono regex date_time filesystem program_options iostreams)
include_directories(${SYSTEM_OPTION} ${Boost_INCLUDE_DIRS})
set(ALL_BOOST_LIBRARIES ${Boost_LIBRARIES})
find_package(Boost OPTIONAL_COMPONENTS atomic timer)
include_directories(${SYSTEM_OPTION} ${Boost_INCLUDE_DIRS})
list(APPEND ALL_BOOST_LIBRARIES ${Boost_LIBRARIES})

#--------------------------------------
# Ceres dependency
#--------------------------------------
option(ENABLE_ceres "Ceres is required for filters using non-linear least square optimization (Slam, autocalibration, ...)" OFF)
if (ENABLE_ceres)
  find_package(Ceres REQUIRED)
  include_directories(${SYSTEM_OPTION} ${CERES_INCLUDE_DIRS})
endif(ENABLE_ceres)

#--------------------------------------
# Opencv dependency
#--------------------------------------
option(ENABLE_opencv "OpenCV is required for handling lidar-camera multisensor systems" OFF)
if (ENABLE_opencv)
  find_package(OpenCV REQUIRED)
  include_directories(${SYSTEM_OPTION} ${OpenCV_INCLUDE_DIRS})
endif(ENABLE_opencv)

#--------------------------------------
# nanoflann dependency
#--------------------------------------
option(ENABLE_nanoflann "Nanoflann will be required for filters using some optimized kdtree (SLAM, DBSCAN, ...)" OFF)
  if (ENABLE_nanoflann)
  # because nanoflann use modern C++, no need to specify the include dir or anything else
  # you just need to link against the target nanoflann::nanoflann
  find_package(nanoflann REQUIRED)
  list(APPEND deps nanoflann::nanoflann)
endif(ENABLE_nanoflann)

#-----------------------------------------------------------------------------
# Build Paraview Plugin
#-----------------------------------------------------------------------------
# The plugin contain both server code and gui code.
# It's possible to only build the server code and skip the gui code
# Currently it's not working well...

# gui specific code
if(PARAVIEW_BUILD_QT_GUI)

  set(UI_FILES
    Ui/vvCalibrationDialog.ui
    Ui/vvCropReturnsDialog.ui
    Ui/vvLaserSelectionDialog.ui
    Ui/vvSelectFramesDialog.ui
    )

  set(MOC_HEADERS
    Ui/vvCalibrationDialog.h
    Ui/vvCropReturnsDialog.h
    Ui/vvLaserSelectionDialog.h
    Ui/vvSelectFramesDialog.h
    ctk/ctkValueProxy.h
    ctk/ctkRangeSlider.h
    ctk/ctkDoubleRangeSlider.h
    )

  set(resources
    Ui/images/resources.qrc)

  # call some qt macro to create moc and code
  qt5_wrap_cpp(moc_srcs ${MOC_HEADERS})
  qt5_wrap_ui(ui_srcs ${UI_FILES})
  qt5_add_resources(resource_srcs ${resources})

  list(APPEND gui_sources
    ${moc_srcs}
    ${ui_srcs}
    ${resource_srcs}
    Ui/vvCalibrationDialog.cxx
    Ui/vvCropReturnsDialog.cxx
    Ui/vvLaserSelectionDialog.cxx
    Ui/vvSelectFramesDialog.cxx
    ctk/ctkPimpl.h
    ctk/ctkCoreExport.h
    ctk/ctkRangeSlider.cpp
    ctk/ctkWidgetsExport.h
    ctk/ctkDoubleRangeSlider.cpp
    ctk/ctkValueProxy.cpp
    )

  # TODO I don"t get this dependancy is for
  list(APPEND deps
    pqApplicationComponents
    )

endif()

# sources that will be wrap by paraview
# this sources will also be wrap in python in another plugin
# WARNING in case of inheritance, put also the upper classes
list(APPEND servermanager_sources
  ${CMAKE_CURRENT_SOURCE_DIR}/IO/Lidar/Common/vtkLidarProvider.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/IO/Lidar/Common/vtkLidarReader.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/IO/Lidar/Common/vtkLidarStream.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/IO/Lidar/Common/vtkLidarPacketInterpreter.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/IO/Lidar/Velodyne/vtkVelodynePacketInterpreter.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/IO/Lidar/KITTIDataSet/vtkLidarKITTIDataSetReader.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/IO/GPS-IMU/Velodyne/vtkVelodyneHDLPositionReader.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/IO/GPS-IMU/Applanix/vtkApplanixPositionReader.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/IO/GPS-IMU/ArduPilotDataFlashLogReader/vtkArduPilotDataFlashLogReader.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/IO/vtkTemporalTransformsReader.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/IO/vtkTemporalTransformsWriter.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/IO/vtkLASFileWriter.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/IO/vtkBoundingBoxReader.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/Filter/MotionDetector/vtkMotionDetector.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/Filter/BirdEyeViewSnap/vtkBirdEyeViewSnap.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/Filter/CameraProjector/vtkCameraProjector.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/Filter/LidarRawSignalImage/vtkLidarRawSignalImage.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/Filter/PointCloudLinearProjector/vtkPointCloudLinearProjector.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/Filter/LaplacianInfilling/vtkLaplacianInfilling.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/Filter/MLSPosesSmoothing/vtkMLSPosesSmoothing.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/Filter/ProcessingSample/vtkProcessingSample.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/Filter/Ransac/vtkRansacPlaneModel.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/Filter/TemporalTransformsApplier/vtkTemporalTransformsApplier.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/Filter/TrailingFrame/vtkTrailingFrame.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/Source/Grid/vtkGridSource.cxx
  )

if (ENABLE_ceres)
  list(APPEND servermanager_sources
    ${CMAKE_CURRENT_SOURCE_DIR}/Filter/CalibrationFromPoses/vtkCalibrationFromPoses.cxx
    ${CMAKE_CURRENT_SOURCE_DIR}/Filter/TemporalTransformsRemapper/vtkTemporalTransformsRemapper.cxx
    )
endif(ENABLE_ceres)

if (ENABLE_pcl)
  list(APPEND servermanager_sources
    ${CMAKE_CURRENT_SOURCE_DIR}/Filter/PCLRansacModel/vtkPCLRansacModel.cxx
    )

endif(ENABLE_pcl)
if (ENABLE_pcl AND ENABLE_ceres AND ENABLE_nanoflann)
  list(APPEND servermanager_sources
    ${CMAKE_CURRENT_SOURCE_DIR}/Filter/Slam/vtkSlam.cxx
    ${CMAKE_CURRENT_SOURCE_DIR}/Filter/Slam/vtkSpinningSensorKeypointExtractor.cxx
    ${CMAKE_CURRENT_SOURCE_DIR}/Filter/Slam/vtkSlamManager.cxx
    )
  list(APPEND sources_which_do_not_inherit_from_vtkObject
    ${CMAKE_CURRENT_SOURCE_DIR}/Filter/Slam/Slam.cxx
    ${CMAKE_CURRENT_SOURCE_DIR}/Filter/Slam/SpinningSensorKeypointExtractor.cxx
    )
endif(ENABLE_pcl AND ENABLE_ceres AND ENABLE_nanoflann)

if (ENABLE_opencv)
  list(APPEND servermanager_sources
    ${CMAKE_CURRENT_SOURCE_DIR}/IO/Camera/vtkPCAPImageReader.cxx
    ${CMAKE_CURRENT_SOURCE_DIR}/IO/Camera/vtkOpenCVVideoReader.cxx
    )
  list(APPEND servermanager_xml
    xml/PCAPImageReader.xml
    )
  list(APPEND sources_which_do_not_inherit_from_vtkObject
    ${CMAKE_CURRENT_SOURCE_DIR}/Common/vtkOpenCVConversions.cxx
    ${CMAKE_CURRENT_SOURCE_DIR}/Common/vtkOpenCVConversions.cxx
    )
endif(ENABLE_opencv)

# xml file that describe the proxy of the wrap sources in paraview
list(APPEND servermanager_xml
  xml/Lidar.xml
  xml/VelodyneLidarPacketInterpreter.xml
  xml/LidarKITTIDataSetReader.xml
  xml/VelodyneHDLPositionReader.xml
  xml/ApplanixPositionReader.xml
  xml/ArduPilotDataFlashLogReader.xml
  xml/BoundingBoxReader.xml
  xml/MotionDetector.xml
  xml/BirdEyeViewSnap.xml
  xml/LidarRawSignalImage.xml
  xml/PointCloudLinearProjector.xml
  xml/LaplacianInfilling.xml
  xml/MLSPosesSmoothing.xml
  xml/RansacPlaneModel.xml
  xml/TrailingFrame.xml
  xml/ProcessingSample.xml
  xml/CameraProjector.xml
  xml/GridSource.xml
  xml/TemporalTransformsReader.xml
  xml/TemporalTransformsWriter.xml
  xml/TemporalTransformsApplier.xml
  xml/TemporalTransformsRemapper.xml
  xml/LASFileWriter.xml
  xml/OpenCVVideoReader.xml
  )

if (ENABLE_pcl)
  list(APPEND servermanager_xml
    xml/PCLRansacModel.xml
    )
endif(ENABLE_pcl)

if (ENABLE_ceres)
  list(APPEND servermanager_xml
    xml/CalibrationFromPoses.xml
    )
endif(ENABLE_ceres)

if (ENABLE_pcl AND ENABLE_ceres AND ENABLE_nanoflann)
  list(APPEND servermanager_xml
    xml/Slam.xml
    )
endif(ENABLE_pcl AND ENABLE_ceres AND ENABLE_nanoflann)

# other source code than won't be wrap by paraview but are needed to create the plugin
# We make the distinction on that inherit from vtkObject as this one will be also wrap
# in python later on
set(sources_which_inherit_from_vtkObject
  ${CMAKE_CURRENT_SOURCE_DIR}/Common/vtkCustomTransformInterpolator.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/Common/vtkTemporalTransforms.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/Filter/OldPlaneFitter/vtkPlaneFitter.cxx
  )
list(APPEND sources_which_do_not_inherit_from_vtkObject
  ${CMAKE_CURRENT_SOURCE_DIR}/IO/Lidar/Common/CrashAnalysing.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/IO/Lidar/Common/NetworkSource.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/IO/Lidar/Common/PacketReceiver.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/IO/Lidar/Common/PacketFileWriter.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/IO/Lidar/Common/PacketConsumer.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/Common/Network/NetworkPacket.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/IO/Lidar/Velodyne/vtkRollingDataAccumulator.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/IO/GPS-IMU/Common/NMEAParser.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/IO/GPS-IMU/Common/GPSProjectionUtils.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/IO/vtkLASFileWriter.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/IO/LASFileWriter.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/Filter/MotionDetector/vtkSphericalMap.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/Filter/Slam/KalmanFilter.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/Common/Network/vtkPacketFileWriter.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/Common/Network/vvPacketSender.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/Common/vtkEigenTools.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/Common/CameraProjection.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/Common/Calib/Camera/CameraModel.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/${interpolator_pach_until_vtk_update}
  ${CMAKE_CURRENT_SOURCE_DIR}/Common/vtkConversions.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/Common/Calib/Temporal/vtkTimeCalibration.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/Common/Calib/Geometric/vtkCarGeometricCalibration.cxx
  )
if (ENABLE_pcl)
  list(APPEND sources_which_do_not_inherit_from_vtkObject
    ${CMAKE_CURRENT_SOURCE_DIR}/Common/vtkPCLConversions.cxx
    ${CMAKE_CURRENT_SOURCE_DIR}/Common/RegistrationTools.cxx
    )
endif(ENABLE_pcl)
if (ENABLE_ceres)
  list(APPEND sources_which_do_not_inherit_from_vtkObject
      ${CMAKE_CURRENT_SOURCE_DIR}/Common/Calib/Geometric/vtkGeometricCalibration.cxx
      ${CMAKE_CURRENT_SOURCE_DIR}/Common/Calib/Camera/CameraCalibration.cxx
      ${CMAKE_CURRENT_SOURCE_DIR}/Filter/Slam/MotionModel/MotionModel.cxx
      )
endif (ENABLE_ceres)
if (ENABLE_pcl AND ENABLE_ceres)
  list(APPEND sources_which_do_not_inherit_from_vtkObject
      ${CMAKE_CURRENT_SOURCE_DIR}/Filter/Slam/TrajectoryReoptimization.cxx
      )
endif(ENABLE_pcl AND ENABLE_ceres)

if (ENABLE_pcl AND ENABLE_ceres AND ENABLE_opencv)
  list(APPEND sources_which_do_not_inherit_from_vtkObject
    ${CMAKE_CURRENT_SOURCE_DIR}/Common/Calib/CameraLidar/MIDHOG.cxx
    )
endif(ENABLE_pcl AND ENABLE_ceres AND ENABLE_opencv)

# plugin dependencies
list(APPEND deps
  ${PCAP_LIBRARY}
  ${liblas_LIBRARY}
  ${ALL_BOOST_LIBRARIES}
  "vtkIOInfovis" # https://public.kitware.com/pipermail/paraview/2016-January/036010.html
  ${vtklibproj4_LIBRARIES}
  ${PCL_LIBRARIES}
  ${CERES_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
  )

# folder where to look for header file
set(plugin_include_dirs
  ${CMAKE_CURRENT_SOURCE_DIR}/Common
  ${CMAKE_CURRENT_SOURCE_DIR}/Common/ML
  ${CMAKE_CURRENT_SOURCE_DIR}/Common/Network
  ${CMAKE_CURRENT_SOURCE_DIR}/Common/Calib/Camera
  ${CMAKE_CURRENT_SOURCE_DIR}/Common/Calib/Geometric
  ${CMAKE_CURRENT_SOURCE_DIR}/Common/Calib/Temporal
  ${CMAKE_CURRENT_SOURCE_DIR}/Common/Calib/CameraLidar
  ${CMAKE_CURRENT_SOURCE_DIR}/Common/NanoflannAdaptor
  ${CMAKE_CURRENT_SOURCE_DIR}/IO/Camera
  ${CMAKE_CURRENT_SOURCE_DIR}/IO/Lidar/Common
  ${CMAKE_CURRENT_SOURCE_DIR}/IO/Lidar/KITTIDataSet
  ${CMAKE_CURRENT_SOURCE_DIR}/IO/Lidar/Velodyne
  ${CMAKE_CURRENT_SOURCE_DIR}/IO/Camera
  ${CMAKE_CURRENT_SOURCE_DIR}/IO/GPS-IMU/Applanix
  ${CMAKE_CURRENT_SOURCE_DIR}/IO/GPS-IMU/ArduPilotDataFlashLogReader/
  ${CMAKE_CURRENT_SOURCE_DIR}/IO/GPS-IMU/Velodyne
  ${CMAKE_CURRENT_SOURCE_DIR}/IO/GPS-IMU/Common
  ${CMAKE_CURRENT_SOURCE_DIR}/IO
  ${CMAKE_CURRENT_SOURCE_DIR}/Filter/CalibrationFromPoses
  ${CMAKE_CURRENT_SOURCE_DIR}/Filter/MotionDetector
  ${CMAKE_CURRENT_SOURCE_DIR}/Filter/BirdEyeViewSnap
  ${CMAKE_CURRENT_SOURCE_DIR}/Filter/CameraProjector
  ${CMAKE_CURRENT_SOURCE_DIR}/Filter/LidarRawSignalImage
  ${CMAKE_CURRENT_SOURCE_DIR}/Filter/PointCloudLinearProjector
  ${CMAKE_CURRENT_SOURCE_DIR}/Filter/LaplacianInfilling
  ${CMAKE_CURRENT_SOURCE_DIR}/Filter/MLSPosesSmoothing
  ${CMAKE_CURRENT_SOURCE_DIR}/Filter/OldPlaneFitter
  ${CMAKE_CURRENT_SOURCE_DIR}/Filter/Ransac
  ${CMAKE_CURRENT_SOURCE_DIR}/Filter/TrailingFrame
  ${CMAKE_CURRENT_SOURCE_DIR}/Filter/TemporalTransformsApplier
  ${CMAKE_CURRENT_SOURCE_DIR}/Filter/ProcessingSample
  ${CMAKE_CURRENT_SOURCE_DIR}/Filter/PCLRansacModel
  ${CMAKE_CURRENT_SOURCE_DIR}/Filter/Slam
  ${CMAKE_CURRENT_SOURCE_DIR}/Filter/Slam/MotionModel
  ${CMAKE_CURRENT_SOURCE_DIR}/Filter/TemporalTransformsRemapper
  ${CMAKE_CURRENT_SOURCE_DIR}/Filter/TrajectoryReoptimization
  ${CMAKE_CURRENT_SOURCE_DIR}/Source/Grid
  )

# give default target name if not specify otherwise
if (NOT VV_PLUGIN_LIBRARY)
  set(VV_PLUGIN_LIBRARY LidarPlugin) # lidarview plugin
endif()

# create pluging
add_paraview_plugin(${VV_PLUGIN_LIBRARY} "1.0"
  SERVER_MANAGER_SOURCES ${servermanager_sources}
  SERVER_MANAGER_XML ${servermanager_xml}
  SERVER_SOURCES ${sources_which_inherit_from_vtkObject} ${sources_which_do_not_inherit_from_vtkObject}
  GUI_SOURCES ${gui_sources}
  REQUIRED_ON_SERVER)
target_include_directories(${VV_PLUGIN_LIBRARY} PRIVATE ${plugin_include_dirs})
target_link_libraries(${VV_PLUGIN_LIBRARY} LINK_PUBLIC ${deps})
if(WIN32)
  # -DBOOST_PROGRAM_OPTIONS_DYN_LINK is necessary to use boost as a dynamic library
  # more about it here: https://svn.boost.org/trac10/ticket/13326
  target_compile_definitions(${VV_PLUGIN_LIBRARY} PRIVATE -DWIN32 -DBOOST_PROGRAM_OPTIONS_DYN_LINK=1)
endif(WIN32)
if (APPLE)
  target_compile_definitions(${VV_PLUGIN_LIBRARY} PRIVATE -DBOOST_PROGRAM_OPTIONS_DYN_LINK=1)
endif(APPLE)

# The main application executable uses a "forward exectuable"
# that sets LD_LIBRARY_PATH to the appropriate location before calling the
# original executable, so that dynamic libraries can be found.
# For the executanles of the standalone tools it is simpler to modify rpaths:
set(CMAKE_SKIP_RPATH FALSE)
SET(CMAKE_SKIP_BUILD_RPATH  FALSE)
SET(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
SET(CMAKE_INSTALL_RPATH "$ORIGIN/../${VV_INSTALL_LIBRARY_DIR}")
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

#-----------------------------------------------------------------------------
# Build PacketFileSender target which enable to playback a recorded pcap on the network
#-----------------------------------------------------------------------------

add_executable(PacketFileSender StandAloneTools/PacketFileSender.cxx)
target_include_directories(PacketFileSender PRIVATE Common/Network)
target_link_libraries(PacketFileSender LINK_PUBLIC ${VV_PLUGIN_LIBRARY} ${ALL_BOOST_LIBRARIES})
if(WIN32)
  target_compile_definitions(PacketFileSender PRIVATE -DWIN32 -DBOOST_PROGRAM_OPTIONS_DYN_LINK=1)
endif(WIN32)

#-----------------------------------------------------------------------------
# Build StandAlone targets which provide different thirparty tools
#-----------------------------------------------------------------------------
if (ENABLE_opencv)
  add_executable(BBoxFromImagesDetections StandAloneTools/BBoxFromImagesDetections.cxx)
  target_include_directories(BBoxFromImagesDetections PRIVATE ${plugin_include_dirs})
  target_link_libraries(BBoxFromImagesDetections LINK_PUBLIC ${VV_PLUGIN_LIBRARY} ${ALL_BOOST_LIBRARIES})
  if(WIN32)
    target_compile_definitions(BBoxFromImagesDetections PRIVATE -DWIN32 -DBOOST_PROGRAM_OPTIONS_DYN_LINK=1)
  endif(WIN32)
endif (ENABLE_opencv)
#-----------------------------------------------------------------------------
# As we don't want our paraview pluging to have a dependancies to PythonQt we
# we create another target which contain all the code "glue" code.
#-----------------------------------------------------------------------------
set(MOC_HEADERS
  pqLidarViewManager.h
  vvPythonQtDecorators.h
  )

set(moc_srcs)
qt5_wrap_cpp(moc_srcs "${MOC_HEADERS}")

set(gui_sources)
list(APPEND gui_sources
  ${moc_srcs}
  pqLidarViewManager.cxx
  )

# give default target name if not specify otherwise
if (NOT VV_NONE_PLUGIN_LIBRARY)
  set(VV_NONE_PLUGIN_LIBRARY LidarPythonQT)
endif()

add_library(${VV_NONE_PLUGIN_LIBRARY} ${gui_sources})
target_include_directories(${VV_NONE_PLUGIN_LIBRARY} PRIVATE ${plugin_include_dirs})
target_link_libraries(${VV_NONE_PLUGIN_LIBRARY} ${VV_PLUGIN_LIBRARY} ${PYTHONQT_LIBRARIES})

#-----------------------------------------------------------------------------
# This code is here for LEGACY REASON and should NOT BE USED and even REMOVE at some point !!!!
# It's actually nice to expose the classes in python, but the more freedom we have
# the more dirty the code get...
# Create a static and dynamic python target named ${VV_PLUGIN_LIBRARY}Python and
# (${VV_PLUGIN_LIBRARY}PythonD that wrap all the classes that inherit from vtk in python.
# When a paraview plugin is created, you only have acces to the Filter exposed
# with a proxy, and you can but SHOULDN'T used the method GetClientSideObject()
# to have acces to the vtkObject directly.
# With what is done here you have acces to all the vtk inherit  C++ code, even if
# it's not exposed in paraview.
# Simply used "import ${VV_PLUGIN_LIBRARY}Python" and you have acces to the
# wrapped c++ code
#-----------------------------------------------------------------------------

# needed as we can't use target_include_directories because the target are not defined yet
include_directories(${plugin_include_dirs})
set(towrap
  ${servermanager_sources}
  ${sources_which_inherit_from_vtkObject}
  )
# the quoted "" are needed otherwise only the first file of the list will be process
WRAP_PLUGIN_FOR_PYTHON(${VV_PLUGIN_LIBRARY} "${towrap}" "NOTTHERE")

#-----------------------------------------------------------------------------
# Install targets and ressources
#-----------------------------------------------------------------------------

set(libraries_to_install
  ${VV_PLUGIN_LIBRARY}
  ${VV_NONE_PLUGIN_LIBRARY}
  ${VV_PLUGIN_LIBRARY}Python
  ${VV_PLUGIN_LIBRARY}PythonD
  )

set(executables_to_install
  PacketFileSender
  )

if (ENABLE_opencv)
  list(APPEND executables_to_install
    BBoxFromImagesDetections
    )
endif (ENABLE_opencv)

if (APPLE)

  # install libraries
  install(TARGETS ${libraries_to_install}
          RUNTIME DESTINATION ${VV_INSTALL_RUNTIME_DIR}/${SOFTWARE_NAME}.app/Contents/Libraries
          LIBRARY DESTINATION ${VV_INSTALL_RUNTIME_DIR}/${SOFTWARE_NAME}.app/Contents/Libraries
          COMPONENT Runtime)

  # install executables
  install(TARGETS ${executables_to_install}
          RUNTIME DESTINATION ${VV_INSTALL_RUNTIME_DIR}/${SOFTWARE_NAME}.app/Contents/bin
          LIBRARY DESTINATION ${VV_INSTALL_RUNTIME_DIR}/${SOFTWARE_NAME}.app/Contents/Libraries
          COMPONENT Runtime)

  # install paraview python modules
  install(DIRECTORY ${ParaView_DIR}/lib/site-packages/paraview DESTINATION ${VV_INSTALL_RUNTIME_DIR}/${SOFTWARE_NAME}.app/Contents/Python)
  file(GLOB python_libs ${ParaView_DIR}/lib/*Python.so)
  install(FILES ${python_libs} DESTINATION ${VV_INSTALL_RUNTIME_DIR}/${SOFTWARE_NAME}.app/Contents/Libraries)

  # install some pdf
  install(FILES ${LidarView_SOURCE_DIR}/Documentation/LidarView_Developer_Guide.pdf DESTINATION ${VV_INSTALL_RUNTIME_DIR}/${SOFTWARE_NAME}.app/Contents/Resources)
  configure_file(${LidarView_SOURCE_DIR}/Documentation/LidarView_Developer_Guide.pdf ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${SOFTWARE_NAME}.app/Contents/Resources/LidarView_Developer_Guide.pdf)

  install(FILES ${LidarView_SOURCE_DIR}/Documentation/LidarView_User_Guide.pdf DESTINATION ${VV_INSTALL_RUNTIME_DIR}/${SOFTWARE_NAME}.app/Contents/Resources)
  configure_file(${LidarView_SOURCE_DIR}/Documentation/LidarView_User_Guide.pdf ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${SOFTWARE_NAME}.app/Contents/Resources/LidarView_User_Guide.pdf)

else()
  # install libraries
  install(TARGETS ${libraries_to_install}
          RUNTIME DESTINATION ${VV_INSTALL_RUNTIME_DIR}
          LIBRARY DESTINATION ${VV_INSTALL_LIBRARY_DIR}
          COMPONENT Runtime)

  # install executables
  install(TARGETS ${executables_to_install}
          RUNTIME DESTINATION ${VV_INSTALL_RUNTIME_DIR}
          LIBRARY DESTINATION ${VV_INSTALL_LIBRARY_DIR}
          COMPONENT Runtime)
endif()

#-----------------------------------------------------------------------------
# Add python source
#-----------------------------------------------------------------------------

add_subdirectory(python)

#-----------------------------------------------------------------------------
# Option to build some tests
#-----------------------------------------------------------------------------

option(BUILD_TESTING "Build some unitary test" OFF)
if(BUILD_TESTING)
  add_subdirectory(Testing)
endif()
