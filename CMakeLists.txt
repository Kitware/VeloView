cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)
project(VelodyneViewer)

find_package(ParaView REQUIRED)
include(${PARAVIEW_USE_FILE})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
if(UNIX)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
else()
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
endif()
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

set (source_files
  vvLoadDataReaction.cxx
  vvLoadDataReaction.h
  vvMainWindow.cxx
  vvMainWindow.h
  )

qt4_wrap_cpp(moc_source_files
  vvLoadDataReaction.h
  vvMainWindow.h
  )

qt4_wrap_ui(ui_source_files
  vvMainWindow.ui
  )

qt4_add_resources(rcs_source_files
  vvResources.qrc)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR})


# Add the reader plugin and any other plugins.
add_subdirectory(VelodyneHDL)

build_paraview_client(VelodyneViewer
  TITLE "LIDAR Viewer - Velodyne"
  ORGANIZATION "Velodyne"
  VERSION_MAJOR 0
  VERSION_MINOR 0
  VERSION_PATCH 1
  PVMAIN_WINDOW vvMainWindow
  PVMAIN_WINDOW_INCLUDE vvMainWindow.h
  BUNDLE_ICON "${CMAKE_CURRENT_SOURCE_DIR}/logo.icns"
  APPLICATION_ICON "${CMAKE_CURRENT_SOURCE_DIR}/logo.ico"
  GUI_CONFIGURATION_XMLS
    "${CMAKE_CURRENT_SOURCE_DIR}/vvReaders.xml"
  SOURCES
    ${source_files}
    ${moc_source_files}
    ${ui_source_files}
    ${rcs_source_files}
)

# We directly link against the plugin and import them statically.
target_link_libraries(VelodyneViewer VelodyneHDLPlugin)
