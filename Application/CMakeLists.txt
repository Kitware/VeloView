include("SoftwareInformation/branding.cmake")

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

build_paraview_client(${SOFTWARE_NAME}
  TITLE "${SOFTWARE_NAME} ${VV_VERSION_FULL} ${VV_BUILD_ARCHITECTURE}-bit"
  ORGANIZATION "${VENDOR}"
  VERSION_MAJOR ${VV_VERSION_MAJOR}
  VERSION_MINOR ${VV_VERSION_MINOR}
  VERSION_PATCH ${VV_VERSION_PATCH}
  PVMAIN_WINDOW vvMainWindow
  PVMAIN_WINDOW_INCLUDE vvMainWindow.h
  SPLASH_IMAGE "${CMAKE_CURRENT_SOURCE_DIR}/SoftwareInformation/${PARAVIEW_SPLASH_IMAGE}"
  # right now the bundle's logo.icns file must be at the root of LidarView sources
  # this problem is due to paraview/src/CMake/ParaViewBranding.cmake,
  # on line: GET_FILENAME_COMPONENT(bundle_icon_file "${BPC_BUNDLE_ICON}" NAME)
  BUNDLE_ICON "${CMAKE_CURRENT_SOURCE_DIR}/${PARAVIEW_BUNDLE_ICON}"
  APPLICATION_ICON "${CMAKE_CURRENT_SOURCE_DIR}/SoftwareInformation/${PARAVIEW_APPLICATION_ICON}"
  SOURCES ${source_files}
  GUI_CONFIGURATION_XMLS
    ${CMAKE_CURRENT_SOURCE_DIR}/vvSources.xml
    ${CMAKE_CURRENT_SOURCE_DIR}/vvFilters.xml
  INSTALL_BIN_DIR ${VV_INSTALL_RUNTIME_DIR}
  INSTALL_LIB_DIR ${VV_INSTALL_LIBRARY_DIR}
)

target_include_directories(${SOFTWARE_NAME} PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR})

# We directly link against the plugin and import them statically.
target_link_libraries(${SOFTWARE_NAME}
  PUBLIC
    LidarPlugin
    ApplicationUi
    VelodynePlugin
    ${PYTHONQTPLUGIN_LIBRARY}
    ${PYTHONQT_LIBRARIES}
    lqApplicationComponents)