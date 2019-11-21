set(server_manager_xml
  VelodyneHDLPositionReader/VelodyneHDLPositionReader.xml
  VelodynePacketInterpreter/VelodyneLidarPacketInterpreter.xml
  )

set(server_manager_sources
  ${CMAKE_CURRENT_SOURCE_DIR}/VelodyneHDLPositionReader/vtkVelodyneHDLPositionReader.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/VelodynePacketInterpreter/vtkVelodynePacketInterpreter.cxx
  ${CMAKE_CURRENT_SOURCE_DIR}/PlaneFitter/vtkPlaneFitter.cxx
  )

set(server_sources
  ${CMAKE_CURRENT_SOURCE_DIR}/VelodynePacketInterpreter/vtkRollingDataAccumulator.cxx
  )

ADD_PARAVIEW_PLUGIN(VelodynePlugin "1.0"
  REQUIRED_ON_SERVER
  SERVER_MANAGER_XML ${server_manager_xml}
  SERVER_MANAGER_SOURCES ${server_manager_sources}
  SERVER_SOURCES)

target_link_libraries(VelodynePlugin PUBLIC LidarPlugin)
target_include_directories(VelodynePlugin
  PUBLIC
  PlaneFitter/
  VelodyneHDLPositionReader/
  VelodynePacketInterpreter/
  )

# Install data files
#set (VV_SHARED_FILES VLP-16.xml HDL-32.xml VLP-32c.xml "Puck Hi-Res.xml" "Puck LITE.xml")
#if(NOT APPLE)
#  set(vv_configurefile_dest "${CMAKE_CURRENT_BINARY_DIR}/share/")
#  set(vv_installfile_dest share)
#else()
#  set(vv_configurefile_dest "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${SOFTWARE_NAME}.app/Contents/Resources/")
#  set(vv_installfile_dest "${VV_INSTALL_RUNTIME_DIR}/${SOFTWARE_NAME}.app/Contents/Resources")
#endif()
#set(vv_installed_xml_files "")
#foreach(vv_xml_file ${VV_SHARED_FILES})
#  configure_file("share/${vv_xml_file}" "${vv_configurefile_dest}${vv_xml_file}" COPYONLY)
#  list(APPEND vv_installed_xml_files "share/${vv_xml_file}")
#endforeach()
#install(FILES ${vv_installed_xml_files} DESTINATION ${vv_installfile_dest})
#unset(VV_SHARED_FILES)
#unset(vv_installed_xml_files)
#unset(vv_configurefile_dest)
#unset(vv_installfile_dest)
