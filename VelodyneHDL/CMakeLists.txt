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



find_library(PCAP_LIBRARY pcap DOC "pcap library")
find_path(PCAP_INCLUDE_DIR pcap.h DOC "pcap include directory")
mark_as_advanced(PCAP_LIBRARY PCAP_INCLUDE_DIR)
include_directories(${PCAP_INCLUDE_DIR})


set(Boost_USE_STATIC_LIBS   ON)
set(Boost_USE_MULTITHREADED ON)
find_package(Boost REQUIRED COMPONENTS system thread)


########################
# build paraview plugin

set(gui_sources)
set(gui_interfaces)
set(gui_xml)
set(servermanager_sources)
set(servermanager_xml)

if(PARAVIEW_BUILD_QT_GUI)
  QT4_WRAP_CPP(moc_srcs pqVelodyneHDLSource.h)
  ADD_PARAVIEW_OBJECT_PANEL(
    ifaces iface_srcs
    CLASS_NAME pqVelodyneHDLSource
    XML_NAME VelodyneHDLSource
    XML_GROUP sources)
  list(APPEND gui_interfaces ${ifaces})
  list(APPEND gui_sources ${iface_srcs} ${moc_srcs})
endif()


list(APPEND servermanager_sources
  vtkVelodyneHDLReader.cxx
  vtkVelodyneHDLSource.cxx
  )


list(APPEND servermanager_xml
  xml/VelodyneHDLReader.xml
  xml/VelodyneHDLSource.xml
  )


list(APPEND gui_xml
  xml/VelodyneHDLReadersGUI.xml
  xml/VelodyneHDLSourcesGUI.xml
  )

list(APPEND deps
  ${PCAP_LIBRARY}
  ${Boost_THREAD_LIBRARY}
  ${Boost_SYSTEM_LIBRARY}
  )


set(library_name VelodyneHDLPlugin)

add_paraview_plugin(${library_name} "1.0"
  GUI_INTERFACES ${gui_interfaces}
  GUI_RESOURCE_FILES ${gui_xml}
  GUI_SOURCES ${gui_sources}
  SERVER_MANAGER_XML ${servermanager_xml}
  SERVER_MANAGER_SOURCES ${servermanager_sources}
  )

target_link_libraries(${library_name} ${deps})

add_executable(PacketFileSender PacketFileSender.cxx)
target_link_libraries(PacketFileSender ${deps})

if (NOT APPLE)
  install(TARGETS VelodyneHDLPlugin
          RUNTIME DESTINATION ${VV_INSTALL_RUNTIME_DIR}
          LIBRARY DESTINATION ${VV_INSTALL_LIBRARY_DIR}
          COMPONENT Runtime)
else()
  install(TARGETS VelodyneHDLPlugin
          RUNTIME DESTINATION ${VV_INSTALL_RUNTIME_DIR}/VelodyneViewer.app/Contents/Libraries
          LIBRARY DESTINATION ${VV_INSTALL_RUNTIME_DIR}/VelodyneViewer.app/Contents/Libraries
          COMPONENT Runtime)
endif()
