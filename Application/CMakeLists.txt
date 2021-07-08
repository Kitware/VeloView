# Sanitize checks
if(NOT LV_INSTALL_LIBRARY_DIR )
  message(FATAL_ERROR "LV_INSTALL_LIBRARY_DIR not set")
endif()
if(NOT SOFTWARE_NAME OR NOT SOFTWARE_VENDOR)
  message(FATAL_ERROR "SOFTWARE_NAME or SOFTWARE_VENDOR branding not set")
endif()
if(NOT LV_VERSION_FULL)
  message(FATAL_ERROR "LV_VERSION_ variables not set")
endif()
if(NOT LV_BUILD_ARCHITECTURE)
  message(FATAL_ERROR "LV_BUILD_ARCHITECTURE not set")
endif()

add_subdirectory("Ui/")

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

set (source_files
  vvMainWindow.h
  vvMainWindow.ui
  vvResources.qrc
  vvMainWindow.cxx
  vvMainWindow.h
  )

# Please make sure to adapt the AboutDialog text in the followin file
#  Lidar\python\lidarview\aboutDialog.py
# You also need to change:
#   - bottom_logo.png (bottom logo)

paraview_client_add(
  NAME ${SOFTWARE_NAME}
  NAMESPACE "LidarView"
  VERSION ${LV_VERSION_FULL}
  APPLICATION_NAME "${SOFTWARE_NAME}"
  ORGANIZATION "${SOFTWARE_VENDOR}"
  TITLE "${SOFTWARE_NAME} ${LV_VERSION_FULL} ${LV_BUILD_ARCHITECTURE}-bit"
  SPLASH_IMAGE     "${CMAKE_CURRENT_SOURCE_DIR}/SoftwareInformation/Splash.jpg"
  BUNDLE_ICON      "${CMAKE_CURRENT_SOURCE_DIR}/logo.icns"
  APPLICATION_ICON "${CMAKE_CURRENT_SOURCE_DIR}/SoftwareInformation/logo.ico"

  REQUIRED_PLUGINS LidarPlugin VelodynePlugin

  MAIN_WINDOW_CLASS vvMainWindow
  MAIN_WINDOW_INCLUDE vvMainWindow.h
  SOURCES ${source_files}
  APPLICATION_XMLS
    ${CMAKE_CURRENT_SOURCE_DIR}/vvSources.xml
    ${CMAKE_CURRENT_SOURCE_DIR}/vvFilters.xml

  RUNTIME_DESTINATION ${LV_INSTALL_RUNTIME_DIR}
  LIBRARY_DESTINATION ${LV_INSTALL_LIBRARY_DIR}
)

target_include_directories(${SOFTWARE_NAME} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR})

# We directly link against the plugin and import them statically.
target_link_libraries(${SOFTWARE_NAME} PUBLIC
  Qt5::Core
  Qt5::Widgets
  Qt5::UiTools
  Qt5::Gui
  Qt5::Help
  Qt5::Network
  Qt5::Sql
  Qt5::Svg
  ParaView::pqCore
  ParaView::pqWidgets
  ParaView::pqComponents
  ParaView::pqApplicationComponents
  ParaView::pqPython
  ParaView::pvpythonmodules
  ParaView::PythonInitializer
  PythonQtPlugin # The plugin is needed to be loaded statically
  lqApplicationComponents #actually LVCore/ApplicationComponents
  ApplicationUi
  )

set(USER_GUIDE_SRC "${LidarView_SOURCE_DIR}/Documentation/VeloView_User_Guide.pdf")

if (APPLE)
  install(FILES ${USER_GUIDE_SRC} DESTINATION "${LV_INSTALL_LIBRARY_DIR}/../Resources") # to install tree
  file(COPY ${USER_GUIDE_SRC} DESTINATION "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${SOFTWARE_NAME}.app/Contents/Resources") # to build tree
else ()
  install(FILES ${USER_GUIDE_SRC} DESTINATION "doc") # to install tree
  file(COPY ${USER_GUIDE_SRC} DESTINATION "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/../doc") # to build tree
endif ()
