set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

add_library(VelodyneUI
  Ui/vvCalibrationDialog.cxx
  Ui/vvCalibrationDialog.h
  Ui/vvCalibrationDialog.ui
  Ui/vvCropReturnsDialog.cxx
  Ui/vvCropReturnsDialog.h
  Ui/vvCropReturnsDialog.ui
  Ui/vvLaserSelectionDialog.cxx
  Ui/vvLaserSelectionDialog.h
  Ui/vvLaserSelectionDialog.ui
  Ui/vvSelectFramesDialog.cxx
  Ui/vvSelectFramesDialog.h
  Ui/vvSelectFramesDialog.ui

  pqLidarViewManager.cxx
  pqLidarViewManager.h
  vvLoadDataReaction.cxx
  vvLoadDataReaction.h
  vvPythonQtDecorators.h
  )
target_link_libraries(VelodyneUI PUBLIC LidarPlugin ${PYTHONQT_LIBRARIES} lqApplicationComponents)
target_include_directories(VelodyneUI PUBLIC
  ${PYTHONQT_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/ctk/
  )

# install libraries
if (APPLE)
  install(TARGETS VelodyneUI
          RUNTIME DESTINATION ${VV_INSTALL_RUNTIME_DIR}/${SOFTWARE_NAME}.app/Contents/Libraries
          LIBRARY DESTINATION ${VV_INSTALL_RUNTIME_DIR}/${SOFTWARE_NAME}.app/Contents/Libraries
          COMPONENT Runtime)
else()
  install(TARGETS VelodyneUI
          RUNTIME DESTINATION ${VV_INSTALL_RUNTIME_DIR}
          LIBRARY DESTINATION ${VV_INSTALL_LIBRARY_DIR}
          COMPONENT Runtime)
endif()


#-----------------------------------------------------------------------------
# Add python source
#-----------------------------------------------------------------------------

add_subdirectory(python)
