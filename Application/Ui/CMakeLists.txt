set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

add_library(ApplicationUi
  Widgets/vvCalibrationDialog.cxx
  Widgets/vvCalibrationDialog.h
  Widgets/vvCalibrationDialog.ui
  Widgets/vvCropReturnsDialog.cxx
  Widgets/vvCropReturnsDialog.h
  Widgets/vvCropReturnsDialog.ui
  Widgets/vvLaserSelectionDialog.cxx
  Widgets/vvLaserSelectionDialog.h
  Widgets/vvLaserSelectionDialog.ui
  Widgets/vvSelectFramesDialog.cxx
  Widgets/vvSelectFramesDialog.h
  Widgets/vvSelectFramesDialog.ui
  Widgets/images/resources.qrc

  pqLidarViewManager.cxx
  pqLidarViewManager.h
  lqOpenSensorReaction.cxx
  lqOpenSensorReaction.h
  vvLoadDataReaction.cxx
  vvLoadDataReaction.h
  vvPythonQtDecorators.h
  )
target_link_libraries(ApplicationUi PUBLIC LidarPlugin ${PYTHONQT_LIBRARIES} lqApplicationComponents)
target_include_directories(ApplicationUi PUBLIC
  ${PYTHONQT_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR}
  )
include(GenerateExportHeader)
generate_export_header(ApplicationUi)
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/applicationui_export.h DESTINATION ${LV_INSTALL_LIBRARY_DIR})

# install libraries
if (APPLE)
  install(TARGETS ApplicationUi
          RUNTIME DESTINATION ${LV_INSTALL_LIBRARY_DIR}
          LIBRARY DESTINATION ${LV_INSTALL_LIBRARY_DIR}
          COMPONENT Runtime)
else()
  install(TARGETS ApplicationUi
          RUNTIME DESTINATION "bin"
          LIBRARY DESTINATION ${LV_INSTALL_LIBRARY_DIR}
          COMPONENT Runtime)
endif()


#-----------------------------------------------------------------------------
# Add python source
#-----------------------------------------------------------------------------

add_subdirectory(python)
