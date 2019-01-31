# Copyright 2013 Velodyne Acoustics, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
include("SoftwareInformation/branding.cmake")
project(VeloView)

set (CMAKE_CXX_STANDARD 11)

if (MSVC)
    # warning level all
    add_compile_options(/Wall)
else()
    # warning level all and extra
    add_compile_options(-Wall -Wextra)
endif()

# Disallow in-source build
if ("${VeloView_SOURCE_DIR}"  STREQUAL "${VeloView_BINARY_DIR}")
  message(FATAL_ERROR
    "${SOFTWARE_NAME} requires an out of source Build. "
    "Please create a separate binary directory and run CMake there.")
endif()

#------------------------------------------------------------------------------
# Set a default build type if none was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'RelWithDebInfo' as none was specified.")
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()

#-------------------------------------------------------------------------------
# Doxygen Documentation option
option(BUILD_DOC "Build documentation" OFF)

if(BUILD_DOC)
  # check if Doxygen is installed
  find_package(Doxygen)
  if (DOXYGEN_FOUND)
      # set input and output files
      set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/Documentation/Doxyfile.in)
      set(DOXYGEN_OUT ${CMAKE_CURRENT_BINARY_DIR}/Documentation/Doxyfile)

      # request to configure the file
      configure_file(${DOXYGEN_IN} ${DOXYGEN_OUT} @ONLY)
      message("Doxygen build started")

      # note the option ALL which allows to build the docs together with the application
      add_custom_target( doc_doxygen ALL
          COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_OUT}
          WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
          COMMENT "Generating API documentation with Doxygen"
          VERBATIM )
  else (DOXYGEN_FOUND)
    message("Doxygen need to be installed to generate the doxygen documentation")
  endif (DOXYGEN_FOUND)
endif()

#-------------------------------------------------------------------------------
set (VeloView_CMAKE_DIR "${VeloView_SOURCE_DIR}/CMake")
set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${VeloView_CMAKE_DIR})

include(Git)
include(ParaViewDetermineVersion)
# Sets VV_VERSION_{MAJOR,MINOR,PATCH} using PARAVIEW determine_version
file(STRINGS version.txt version_txt)
extract_version_components("${version_txt}" "VV")
determine_version(${VeloView_SOURCE_DIR} ${GIT_EXECUTABLE} "VV")

# Is this a 32 bit or 64bit build. Display this in about dialog.
if ("${CMAKE_SIZEOF_VOID_P}" EQUAL 8)
  set(VV_BUILD_ARCHITECTURE "64")
else()
  set(VV_BUILD_ARCHITECTURE "32")
endif()

include(CTest)

find_package(ParaView REQUIRED)
find_package(PythonInterp 2 REQUIRED)
find_package(PythonLibs 2 REQUIRED)
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
    lib/veloview-${VV_VERSION_MAJOR}.${VV_VERSION_MINOR})
endif()
set(VV_INSTALL_ARCHIVE_DIR
    lib/veloview-${VV_VERSION_MAJOR}.${VV_VERSION_MINOR})

# Find and include PythonQt
find_package(PythonQt REQUIRED)

# Find the PythonQt plugin library in ParaView lib directory
get_target_property(paraview_lib_location pqCore LOCATION)
get_filename_component(paraview_lib_location ${paraview_lib_location} PATH)
find_library(PYTHONQTPLUGIN_LIBRARY PythonQtPlugin HINTS ${paraview_lib_location}  DOC "ParaView PythonQt plugin library")
mark_as_advanced(PYTHONQTPLUGIN_LIBRARY)

if(WIN32 AND MSVC)
  if(MSVC_VERSION GREATER 1699)
    add_definitions(/D_ALLOW_KEYWORD_MACROS)
  endif()
  add_definitions(/Dinline=__inline)
endif()

set (source_files
  vvMainWindow.cxx
  vvColorToolbar.cxx
  vvToggleSpreadSheetReaction.cxx
  vvLoadDataReaction.cxx
  vvPlayerControlsToolbar.cxx
  vvPlayerControlsController.cxx
  )

qt5_wrap_cpp(moc_source_files
  vvColorToolbar.h
  vvMainWindow.h
  vvToggleSpreadSheetReaction.h
  vvLoadDataReaction.h
  vvPlayerControlsToolbar.h
  vvPlayerControlsController.h
  )

qt5_wrap_ui(ui_source_files
  vvColorToolbar.ui
  vvMainWindow.ui
  vvAboutDialog.ui
  vvPlayerControlsToolbar.ui
  )

qt5_add_resources(rcs_source_files
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
set(VV_PLUGIN_LIBRARY VelodyneHDLPlugin) # veloview plugin
set(VV_NONE_PLUGIN_LIBRARY VelodyneHDLPythonQT) # code needed by veloview but that is not part of the plugin
add_subdirectory(VelodyneHDL)
include_directories(VelodyneHDL)


# Install data files
set (VV_SHARED_FILES VLP-16.xml HDL-32.xml VLP-32c.xml "Puck Hi-Res.xml" "Puck LITE.xml")
if(NOT APPLE)
  set(vv_configurefile_dest "${CMAKE_CURRENT_BINARY_DIR}/share/")
  set(vv_installfile_dest share)
else()
  set(vv_configurefile_dest "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${SOFTWARE_NAME}.app/Contents/Resources/")
  set(vv_installfile_dest "${VV_INSTALL_RUNTIME_DIR}/${SOFTWARE_NAME}.app/Contents/Resources")
endif()
set(vv_installed_xml_files "")
foreach(vv_xml_file ${VV_SHARED_FILES})
  configure_file("share/${vv_xml_file}" "${vv_configurefile_dest}${vv_xml_file}" COPYONLY)
  list(APPEND vv_installed_xml_files "share/${vv_xml_file}")
endforeach()
install(FILES ${vv_installed_xml_files} DESTINATION ${vv_installfile_dest})
unset(VV_SHARED_FILES)
unset(vv_installed_xml_files)
unset(vv_configurefile_dest)
unset(vv_installfile_dest)


# TODO: once we use Qt built from source on Windows, remove this manual installation
# currently: PythonQt fails to build against our own build of Qt
if (WIN32)
  foreach (qt5_opengl_lib IN ITEMS opengl32sw libEGL libGLESv2 libEGLd
      Qt5Core Qt5Gui Qt5Widgets Qt5Help Qt5Network Qt5PrintSupport Qt5Sql)
    install(
       FILES
      "${Qt5_DIR}/../../../bin/${qt5_opengl_lib}.dll"
      DESTINATION
      "bin")
  endforeach ()
  install(DIRECTORY "${Qt5_DIR}/../../../plugins/platforms"
    DESTINATION "bin"
  )
endif ()


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
  # right now the bundle's logo.icns file must be at the root of VeloView sources
  # this problem is due to paraview/src/CMake/ParaViewBranding.cmake,
  # on line: GET_FILENAME_COMPONENT(bundle_icon_file "${BPC_BUNDLE_ICON}" NAME)
  BUNDLE_ICON "${CMAKE_CURRENT_SOURCE_DIR}/${PARAVIEW_BUNDLE_ICON}"
  APPLICATION_ICON "${CMAKE_CURRENT_SOURCE_DIR}/SoftwareInformation/${PARAVIEW_APPLICATION_ICON}"
  SOURCES
    ${source_files}
    ${moc_source_files}
    ${ui_source_files}
    ${rcs_source_files}
  GUI_CONFIGURATION_XMLS
    ${CMAKE_CURRENT_SOURCE_DIR}/vvSources.xml
    ${CMAKE_CURRENT_SOURCE_DIR}/vvFilters.xml
  INSTALL_BIN_DIR ${VV_INSTALL_RUNTIME_DIR}
  INSTALL_LIB_DIR ${VV_INSTALL_LIBRARY_DIR}
)

# We directly link against the plugin and import them statically.
target_link_libraries(${SOFTWARE_NAME} LINK_PUBLIC ${VV_PLUGIN_LIBRARY} ${VV_NONE_PLUGIN_LIBRARY} ${PYTHONQTPLUGIN_LIBRARY} ${PYTHONQT_LIBRARIES})
