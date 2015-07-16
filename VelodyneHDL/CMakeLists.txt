project(VelodyneHDLPlugin)
cmake_minimum_required(VERSION 2.8)

if(ParaView_SOURCE_DIR)

  include_directories(${VTK_INCLUDE_DIRS})
  if(PARAVIEW_BUILD_QT_GUI)
    include(${QT_USE_FILE})
    include_directories(${PARAVIEW_GUI_INCLUDE_DIRS})
  endif(PARAVIEW_BUILD_QT_GUI)

else(ParaView_SOURCE_DIR)
  find_package(ParaView REQUIRED)
  include(${PARAVIEW_USE_FILE})

endif(ParaView_SOURCE_DIR)

vtk_module_load(vtklibproj4)
include_directories(${vtklibproj4_INCLUDE_DIRS})

find_library(PCAP_LIBRARY pcap DOC "pcap library")
find_path(PCAP_INCLUDE_DIR pcap.h DOC "pcap include directory")
mark_as_advanced(PCAP_LIBRARY PCAP_INCLUDE_DIR)
include_directories(${PCAP_INCLUDE_DIR})

set(LASNAME las)
if(WIN32)
  set(LASNAME liblas)
endif()

find_library(liblas_LIBRARY ${LASNAME} DOC "liblas library")
find_path(liblas_INCLUDE_DIR liblas/version.hpp DOC "liblas include directory")
mark_as_advanced(liblas_LIBRARY liblas_INCLUDE_DIR)
include_directories(${liblas_INCLUDE_DIR})

find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIR})

if(WIN32 OR APPLE)
  set(Boost_USE_STATIC_LIBS ON)
endif(WIN32 OR APPLE)
set(Boost_USE_MULTITHREADED ON)
find_package(Boost REQUIRED COMPONENTS system thread chrono regex date_time filesystem)
set(ALL_BOOST_LIBRARIES ${Boost_LIBRARIES})
include_directories(${Boost_INCLUDE_DIRS})
find_package(Boost COMPONENTS atomic)
list(APPEND ALL_BOOST_LIBRARIES ${Boost_LIBRARIES})
include_directories(${Boost_INCLUDE_DIRS})
find_package(Boost COMPONENTS timer)
list(APPEND ALL_BOOST_LIBRARIES ${Boost_LIBRARIES})
include_directories(${Boost_INCLUDE_DIRS})

########################
# build paraview plugin

set(gui_sources)
set(gui_interfaces)
set(gui_xml)
set(servermanager_sources)
set(servermanager_xml)
set(deps)

if(PARAVIEW_BUILD_QT_GUI)

  include_directories(${PYTHONQT_INCLUDE_DIRS})

  set(UI_FILES
    vvCalibrationDialog.ui
    vvCropReturnsDialog.ui
    vvLaserSelectionDialog.ui
    vvSelectFramesDialog.ui
    )

  set(MOC_HEADERS
    pqVelodyneHDLSource.h
    vvCalibrationDialog.h
    vvCropReturnsDialog.h
    vvLaserSelectionDialog.h
    vvSelectFramesDialog.h
    )

  set(resources
    images/resources.qrc)

  set(vtk_sources
    vtkPlaneFitter.cxx
    vtkVelodyneTransformInterpolator.cxx
    vtkWrappedTupleInterpolator.cxx
    )

  set(non_vtk_sources
    vtkLASFileWriter.cxx
    vtkPacketFileWriter.cxx
    vvPacketSender.cxx
    )

  QT4_WRAP_CPP(moc_srcs ${MOC_HEADERS})
  QT4_WRAP_UI(ui_srcs ${UI_FILES})
  QT4_ADD_RESOURCES(resource_srcs ${resources})

  ADD_PARAVIEW_OBJECT_PANEL(
    ifaces iface_srcs
    CLASS_NAME pqVelodyneHDLSource
    XML_NAME VelodyneHDLSource
    XML_GROUP sources)

  list(APPEND gui_interfaces ${ifaces})
  list(APPEND gui_sources
    ${iface_srcs}
    ${moc_srcs}
    ${ui_srcs}
    ${resource_srcs}
    vvCalibrationDialog.cxx
    vvCropReturnsDialog.cxx
    vvLaserSelectionDialog.cxx
    vvSelectFramesDialog.cxx
    )

  list(APPEND deps
    pqApplicationComponents
    ${vtklibproj4_LIBRARIES}
    )

endif()


list(APPEND servermanager_sources
  vtkApplanixPositionReader.cxx
  vtkProcessingSample.cxx
  vtkVelodyneHDLGridSource.cxx
  vtkVelodyneHDLReader.cxx
  vtkVelodyneHDLPositionReader.cxx
  vtkVelodyneHDLSource.cxx)


list(APPEND servermanager_xml
  xml/ApplanixPositionReader.xml
  xml/ProcessingSample.xml
  xml/VelodyneHDLGridSource.xml
  xml/VelodyneHDLReader.xml
  xml/VelodyneHDLPositionReader.xml
  xml/VelodyneHDLSource.xml)

list(APPEND gui_xml
  xml/VelodyneHDLSourcesGUI.xml)

list(APPEND deps
  ${PCAP_LIBRARY}
  ${liblas_LIBRARY}
  ${ALL_BOOST_LIBRARIES}
  )


set(library_name VelodyneHDLPlugin)

add_paraview_plugin(${library_name} "1.0"
  GUI_INTERFACES ${gui_interfaces}
  GUI_RESOURCE_FILES ${gui_xml}
  GUI_SOURCES ${gui_sources} ${vtk_sources}
  SERVER_MANAGER_XML ${servermanager_xml}
  SERVER_MANAGER_SOURCES ${servermanager_sources}
  SOURCES ${non_vtk_sources}
  REQUIRED_ON_SERVER)

target_link_libraries(${library_name} LINK_PUBLIC ${deps})

add_executable(PacketFileSender PacketFileSender.cxx)
target_link_libraries(PacketFileSender LINK_PUBLIC ${library_name})


#########
set(MOC_HEADERS
  pqVelodyneManager.h
  vvLoadDataReaction.h
  vvPythonQtDecorators.h
  )

set(moc_srcs)
QT4_WRAP_CPP(moc_srcs ${MOC_HEADERS})

set(gui_sources)
list(APPEND gui_sources
  ${moc_srcs}
  pqVelodyneManager.cxx
  vvLoadDataReaction.cxx
  )

add_library(VelodyneHDLPythonQT ${gui_sources})
target_link_libraries(VelodyneHDLPythonQT ${library_name} ${PYTHONQT_LIBRARIES})
#########

list(APPEND wrap_sources ${vtk_sources} ${servermanager_sources})

WRAP_PLUGIN_FOR_PYTHON(VelodyneHDLPlugin "${wrap_sources}" "NOTTHERE")

if (NOT APPLE)
  install(TARGETS VelodyneHDLPlugin
          RUNTIME DESTINATION ${VV_INSTALL_RUNTIME_DIR}
          LIBRARY DESTINATION ${VV_INSTALL_LIBRARY_DIR}
          COMPONENT Runtime)

  install(TARGETS VelodyneHDLPluginPython
          RUNTIME DESTINATION ${VV_INSTALL_RUNTIME_DIR}
          LIBRARY DESTINATION ${VV_INSTALL_LIBRARY_DIR}
          COMPONENT Runtime)

  install(TARGETS VelodyneHDLPluginPythonD
          RUNTIME DESTINATION ${VV_INSTALL_RUNTIME_DIR}
          LIBRARY DESTINATION ${VV_INSTALL_LIBRARY_DIR}
          COMPONENT Runtime)

  install(TARGETS VelodyneHDLPythonQT
          RUNTIME DESTINATION ${VV_INSTALL_RUNTIME_DIR}
          LIBRARY DESTINATION ${VV_INSTALL_LIBRARY_DIR}
          COMPONENT Runtime)

else()
  install(TARGETS VelodyneHDLPlugin
          RUNTIME DESTINATION ${VV_INSTALL_RUNTIME_DIR}/VeloView.app/Contents/Libraries
          LIBRARY DESTINATION ${VV_INSTALL_RUNTIME_DIR}/VeloView.app/Contents/Libraries
          COMPONENT Runtime)

  install(TARGETS VelodyneHDLPluginPython
          RUNTIME DESTINATION ${VV_INSTALL_RUNTIME_DIR}/VeloView.app/Contents/Libraries
          LIBRARY DESTINATION ${VV_INSTALL_RUNTIME_DIR}/VeloView.app/Contents/Libraries
          COMPONENT Runtime)

  install(TARGETS VelodyneHDLPluginPythonD
          RUNTIME DESTINATION ${VV_INSTALL_RUNTIME_DIR}/VeloView.app/Contents/Libraries
          LIBRARY DESTINATION ${VV_INSTALL_RUNTIME_DIR}/VeloView.app/Contents/Libraries
          COMPONENT Runtime)

  install(TARGETS VelodyneHDLPythonQT
          RUNTIME DESTINATION ${VV_INSTALL_RUNTIME_DIR}/VeloView.app/Contents/Libraries
          LIBRARY DESTINATION ${VV_INSTALL_RUNTIME_DIR}/VeloView.app/Contents/Libraries
          COMPONENT Runtime)

  # install paraview python modules
  install(DIRECTORY ${ParaView_DIR}/lib/site-packages/paraview DESTINATION ${VV_INSTALL_RUNTIME_DIR}/VeloView.app/Contents/Python)
  file(GLOB python_libs ${ParaView_DIR}/lib/*Python.so)
  install(FILES ${python_libs} DESTINATION ${VV_INSTALL_RUNTIME_DIR}/VeloView.app/Contents/Libraries)

  install(FILES ${VeloView_SOURCE_DIR}/Documentation/VeloView_Developer_Guide.pdf DESTINATION ${VV_INSTALL_RUNTIME_DIR}/VeloView.app/Contents/Resources)
  configure_file(${VeloView_SOURCE_DIR}/Documentation/VeloView_Developer_Guide.pdf ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/VeloView.app/Contents/Resources/VeloView_Developer_Guide.pdf)

  install(FILES ${VeloView_SOURCE_DIR}/Documentation/VeloView_User_Guide.pdf DESTINATION ${VV_INSTALL_RUNTIME_DIR}/VeloView.app/Contents/Resources)
  configure_file(${VeloView_SOURCE_DIR}/Documentation/VeloView_User_Guide.pdf ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/VeloView.app/Contents/Resources/VeloView_User_Guide.pdf)

endif()

add_subdirectory(python)
if(BUILD_TESTING)
  add_subdirectory(Testing)
endif()
