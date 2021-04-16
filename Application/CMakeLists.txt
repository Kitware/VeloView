#Sanitize checks
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
if(NOT PARAVIEW_VERSION_MAJOR OR NOT PARAVIEW_VERSION_MINOR)
  message(FATAL_ERROR "PARAVIEW_VERSION_ not set")
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

# This ensure that build_paraview_client() adds an "forwarding path" to locate
# paraview from installed location.
set (PARAVIEW_INSTALL_LIB_DIR "lib/paraview-${PARAVIEW_VERSION_MAJOR}.${PARAVIEW_VERSION_MINOR}")

# Please make sure to adapt the AboutDialog text in the followin file
#  Lidar\python\lidarview\aboutDialog.py
# You also need to change:
#   - bottom_logo.png (bottom logo)

build_paraview_client(${SOFTWARE_NAME}
  TITLE "${SOFTWARE_NAME} ${LV_VERSION_FULL} ${LV_BUILD_ARCHITECTURE}-bit"
  ORGANIZATION "${SOFTWARE_VENDOR}"
  VERSION_MAJOR ${LV_VERSION_MAJOR}
  VERSION_MINOR ${LV_VERSION_MINOR}
  VERSION_PATCH ${LV_VERSION_PATCH}
  PVMAIN_WINDOW vvMainWindow
  PVMAIN_WINDOW_INCLUDE vvMainWindow.h
  SPLASH_IMAGE "${CMAKE_CURRENT_SOURCE_DIR}/SoftwareInformation/Splash.jpg"
  # right now the bundle's logo.icns file must be at the root of LidarView sources
  # this problem is due to paraview/src/CMake/ParaViewBranding.cmake,
  # on line: GET_FILENAME_COMPONENT(bundle_icon_file "${BPC_BUNDLE_ICON}" NAME)
  BUNDLE_ICON "${CMAKE_CURRENT_SOURCE_DIR}/logo.icns"
  APPLICATION_ICON "${CMAKE_CURRENT_SOURCE_DIR}/SoftwareInformation/logo.ico"
  SOURCES ${source_files}
  GUI_CONFIGURATION_XMLS
    ${CMAKE_CURRENT_SOURCE_DIR}/vvSources.xml
    ${CMAKE_CURRENT_SOURCE_DIR}/vvFilters.xml
  INSTALL_BIN_DIR "bin"
  INSTALL_LIB_DIR ${LV_INSTALL_LIBRARY_DIR}
)

target_include_directories(${SOFTWARE_NAME} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR})

# We directly link against the plugin and import them statically.
if(NOT PYTHONQTPLUGIN_LIBRARY OR NOT PYTHONQT_LIBRARIES)
  message(FATAL_ERROR "PYTHONQTPLUGIN_LIBRARY OR PYTHONQT_LIBRARIES not set")
endif()
target_link_libraries(${SOFTWARE_NAME}
  PUBLIC
    LidarPlugin
    ApplicationUi
    VelodynePlugin
    ${PYTHONQTPLUGIN_LIBRARY}
    ${PYTHONQT_LIBRARIES}
    lqApplicationComponents)

set(USER_GUIDE_SRC "${LidarView_SOURCE_DIR}/Documentation/VeloView_User_Guide.pdf")

if (APPLE)
  install(FILES ${USER_GUIDE_SRC} DESTINATION "${LV_INSTALL_LIBRARY_DIR}/../Resources") # to install tree
  file(COPY ${USER_GUIDE_SRC} DESTINATION "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${SOFTWARE_NAME}.app/Contents/Resources") # to build tree
else ()
  install(FILES ${USER_GUIDE_SRC} DESTINATION "doc") # to install tree
  file(COPY ${USER_GUIDE_SRC} DESTINATION "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/../doc") # to build tree
endif ()
