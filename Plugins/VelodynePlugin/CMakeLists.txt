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
  SERVER_SOURCES ${server_sources}
  REQUIRED_ON_SERVER)

target_link_libraries(VelodynePlugin PUBLIC LidarPlugin)
target_include_directories(VelodynePlugin
  PUBLIC
  PlaneFitter/
  VelodyneHDLPositionReader/
  VelodynePacketInterpreter/
  )

# install libraries
install(TARGETS VelodynePlugin
        RUNTIME DESTINATION ${VV_INSTALL_RUNTIME_DIR}
        LIBRARY DESTINATION ${VV_INSTALL_LIBRARY_DIR}
        COMPONENT Runtime)

# Install data files
set (calib_files
  VLP-16.xml
  HDL-32.xml
  VLP-32c.xml
  "Puck Hi-Res.xml"
  "Puck LITE.xml")

# copy the calibration to the build folder
foreach(file ${calib_files})
  configure_file("CalibrationsFiles/${file}" "${PROJECT_BINARY_DIR}/share/${file}" COPYONLY)
endforeach()

# install step
set(installfile_dest share)
if(APPLE)
  set(installfile_dest "${VV_INSTALL_RUNTIME_DIR}/${SOFTWARE_NAME}.app/Contents/Resources")
endif()
foreach(file ${calib_files})
  install(FILES "${PROJECT_BINARY_DIR}/share/${file}" DESTINATION ${installfile_dest})
endforeach()
