set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

add_library(lqApplicationComponents
  ctk/ctkCoreExport.h
  ctk/ctkDoubleRangeSlider.cpp
  ctk/ctkDoubleRangeSlider.h
  ctk/ctkPimpl.h
  ctk/ctkRangeSlider.cpp
  ctk/ctkRangeSlider.h
  ctk/ctkValueProxy.cpp
  ctk/ctkValueProxy.h
  ctk/ctkWidgetsExport.h
  vvColorToolbar.cxx
  vvColorToolbar.h
  vvColorToolbar.ui
  vvPlayerControlsToolbar.cxx
  vvPlayerControlsToolbar.h
  vvPlayerControlsToolbar.ui
  vvPlayerControlsController.cxx
  vvPlayerControlsController.h
  vvToggleSpreadSheetReaction.h
  vvToggleSpreadSheetReaction.cxx

  )
target_include_directories(lqApplicationComponents PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_SOURCE_DIR}/ctk/
  )
target_link_libraries(lqApplicationComponents PUBLIC pqApplicationComponents)

# install libraries
if (APPLE)
  install(TARGETS lqApplicationComponents
          RUNTIME DESTINATION ${VV_INSTALL_RUNTIME_DIR}/${SOFTWARE_NAME}.app/Contents/Libraries
          LIBRARY DESTINATION ${VV_INSTALL_RUNTIME_DIR}/${SOFTWARE_NAME}.app/Contents/Libraries
          COMPONENT Runtime)
else()
  install(TARGETS lqApplicationComponents
          RUNTIME DESTINATION ${VV_INSTALL_RUNTIME_DIR}
          LIBRARY DESTINATION ${VV_INSTALL_LIBRARY_DIR}
          COMPONENT Runtime)
endif()
