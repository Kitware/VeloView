cmake_minimum_required(VERSION 2.8.8 FATAL_ERROR)
project(VeloView)

set (VV_VERSION_MAJOR 1)
set (VV_VERSION_MINOR 0)
set (VV_VERSION_PATCH 0)

find_package(ParaView REQUIRED)
include(${PARAVIEW_USE_FILE})
option(BUILD_SHARED_LIBS "Build shared libs" ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
if(UNIX)
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")
else()
  set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")
endif()
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/lib")

# Setting this ensures that "make install" will leave rpaths to external
# libraries (not part of the build-tree e.g. Qt, ffmpeg, etc.) intact on
# "make install". This ensures that one can install a version of ParaView on the
# build machine without any issues. If this not desired, simply comment the
# following line and "make install" will strip all rpaths, which is default
# behavior.
SET(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)


#------------------------------------------------------------------------------
# Setup install directories (we use names with VTK_ prefix, since ParaView now
# is built as a custom "VTK" library.
set(VV_INSTALL_RUNTIME_DIR bin)
if (WIN32)
  set(VV_INSTALL_LIBRARY_DIR bin)
else()
  set(VV_INSTALL_LIBRARY_DIR
    lib/velodyne-viewer-${VV_VERSION_MAJOR}.${VV_VERSION_MINOR})
endif()
set(VV_INSTALL_ARCHIVE_DIR
    lib/velodyne-viewer-${VV_VERSION_MAJOR}.${VV_VERSION_MINOR})

set (source_files
  vvLoadDataReaction.cxx
  vvLoadDataReaction.h
  vvMainWindow.cxx
  vvMainWindow.h
  vvSelectionReaction.cxx
  vvSelectionReaction.h
  vvToggleSpreadSheetReaction.cxx
  vvToggleSpreadSheetReaction.h
  )

qt4_wrap_cpp(moc_source_files
  vvLoadDataReaction.h
  vvMainWindow.h
  vvSelectionReaction.h
  vvToggleSpreadSheetReaction.h
  )

qt4_wrap_ui(ui_source_files
  vvMainWindow.ui
  )

qt4_add_resources(rcs_source_files
  vvResources.qrc)

include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}
  ${CMAKE_CURRENT_BINARY_DIR})

if (APPLE)
  set(CMAKE_INSTALL_NAME_DIR "@executable_path/../Libraries")

  # ensure that we don't build forwarding executables on apple.
  set(VTK_BUILD_FORWARDING_EXECUTABLES FALSE)
endif()

# Add the reader plugin and any other plugins.
add_subdirectory(VelodyneHDL)

# This ensure that build_paraview_client() adds an "forwarding path" to locate
# paraview from installed location.
set (PARAVIEW_INSTALL_LIB_DIR "lib/paraview-${PARAVIEW_VERSION_MAJOR}.${PARAVIEW_VERSION_MINOR}")

build_paraview_client(VeloView
  TITLE "VeloView"
  ORGANIZATION "Velodyne Lidar"
  VERSION_MAJOR ${VV_VERSION_MAJOR}
  VERSION_MINOR ${VV_VERSION_MINOR}
  VERSION_PATCH ${VV_VERSION_PATCH}
  PVMAIN_WINDOW vvMainWindow
  PVMAIN_WINDOW_INCLUDE vvMainWindow.h
  SPLASH_IMAGE "${CMAKE_CURRENT_SOURCE_DIR}/Splash.jpg"
  BUNDLE_ICON "${CMAKE_CURRENT_SOURCE_DIR}/logo.icns"
  APPLICATION_ICON "${CMAKE_CURRENT_SOURCE_DIR}/logo.ico"
  GUI_CONFIGURATION_XMLS
    "${CMAKE_CURRENT_SOURCE_DIR}/vvReaders.xml"
  SOURCES
    ${source_files}
    ${moc_source_files}
    ${ui_source_files}
    ${rcs_source_files}
  INSTALL_BIN_DIR ${VV_INSTALL_RUNTIME_DIR}
  INSTALL_LIB_DIR ${VV_INSTALL_LIBRARY_DIR}
)

# We directly link against the plugin and import them statically.
target_link_libraries(VeloView VelodyneHDLPlugin)
