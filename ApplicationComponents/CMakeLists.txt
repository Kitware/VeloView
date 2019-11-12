qt5_wrap_cpp(moc_source_files
  vvColorToolbar.h
  vvToggleSpreadSheetReaction.h
  vvPlayerControlsToolbar.h
  vvPlayerControlsController.h
  )

qt5_wrap_ui(ui_source_files
  vvColorToolbar.ui
  vvPlayerControlsToolbar.ui
  )

set(source_files
  ${moc_source_files}
  ${ui_source_files}
  vvColorToolbar.cxx
  vvToggleSpreadSheetReaction.cxx
  vvPlayerControlsToolbar.cxx
  vvPlayerControlsController.cxx
  )

add_library(lqApplicationComponents ${source_files})
target_include_directories(lqApplicationComponents PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
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
