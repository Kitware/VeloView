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



add_paraview_plugin(VelodyneHDLPlugin "1.0"
  SERVER_MANAGER_XML VelodyneHDLReader.xml
  SERVER_MANAGER_SOURCES vtkVelodyneHDLReader.cxx
  GUI_RESOURCE_FILES VelodyneHDLReaderGUI.xml
)

target_link_libraries(VelodyneHDLPlugin ${PCAP_LIBRARY})
install(TARGETS VelodyneHDLPlugin
        RUNTIME DESTINATION ${VV_INSTALL_RUNTIME_DIR}
        LIBRARY DESTINATION ${VV_INSTALL_LIBRARY_DIR}
        COMPONENT Runtime)
