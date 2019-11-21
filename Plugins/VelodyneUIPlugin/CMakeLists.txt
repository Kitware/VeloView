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

  ctk/ctkCoreExport.h
  ctk/ctkDoubleRangeSlider.cpp
  ctk/ctkDoubleRangeSlider.h
  ctk/ctkPimpl.h
  ctk/ctkRangeSlider.cpp
  ctk/ctkRangeSlider.h
  ctk/ctkValueProxy.cpp
  ctk/ctkValueProxy.h
  ctk/ctkWidgetsExport.h

  pqLidarViewManager.cxx
  pqLidarViewManager.h
  vvLoadDataReaction.cxx
  vvLoadDataReaction.h
  vvPythonQtDecorators.h
  )
target_link_libraries(VelodyneUI PUBLIC LidarPlugin ${PYTHONQT_LIBRARIES})
target_include_directories(VelodyneUI PUBLIC
  ${PYTHONQT_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/ctk/
  )
install(TARGETS VelodyneUI
        RUNTIME DESTINATION lib)

#-----------------------------------------------------------------------------
# Add python source
#-----------------------------------------------------------------------------

add_subdirectory(python)
