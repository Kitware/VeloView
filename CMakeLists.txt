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
project(LidarView)
include("Application/SoftwareInformation/branding.cmake")

set (CMAKE_CXX_STANDARD 14)
set (CMAKE_CXX_EXTENSIONS OFF)
message(STATUS "Building LidarView with C++${CMAKE_CXX_STANDARD} standard")

# Disallow in-source build
if ("${LidarView_SOURCE_DIR}"  STREQUAL "${LidarView_BINARY_DIR}")
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
set (LidarView_CMAKE_DIR "${LidarView_SOURCE_DIR}/LVCore/CMake")
set (CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${LidarView_CMAKE_DIR})

include(SetCompilationWarnings)
include(Git)
include(ParaViewDetermineVersion)
# Sets LV_VERSION_{MAJOR,MINOR,PATCH} using PARAVIEW determine_version
file(STRINGS version.txt version_txt)
extract_version_components("${version_txt}" "LV")
determine_version(${LidarView_SOURCE_DIR} ${GIT_EXECUTABLE} "LV")

# Is this a 32 bit or 64bit build. Display this in about dialog.
if ("${CMAKE_SIZEOF_VOID_P}" EQUAL 8)
  set(LV_BUILD_ARCHITECTURE "64")
else()
  set(LV_BUILD_ARCHITECTURE "32")
endif()

include(CTest)

find_package(ParaView REQUIRED)

# Include findpythonlibs to get the function PYTHON_ADD_MODULE
# needed in WRAP_PLUGIN_FOR_PYTHON
# Warning : Do not use find_package(PythonLibs) because it reset 
# some PYTHON paths that can enter in conflicts with those found
# during the superbuild (specially right on APPLE when a second
# python version has been installed using homebrew)
include(FindPythonLibs)
# We force python to version 3.7 as it is the only one that has been tested
find_package(Python3 3.7 EXACT QUIET REQUIRED COMPONENTS Interpreter)
set(Python_VERSION "${Python3_VERSION_MAJOR}.${Python3_VERSION_MINOR}")

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
if (WIN32)
  set(LV_INSTALL_LIBRARY_DIR bin)
elseif(APPLE)
  set(LV_INSTALL_LIBRARY_DIR bin/${SOFTWARE_NAME}.app/Contents/Libraries)
else()
  set(LV_INSTALL_LIBRARY_DIR lib)
endif()

# Find and include PythonQt
find_package(PythonQt REQUIRED)

# Find the PythonQt plugin library in ParaView lib directory
get_target_property(paraview_lib_location pqCore LOCATION)
get_filename_component(paraview_lib_location ${paraview_lib_location} PATH)

if(WIN32)
  set(PYTHONQTPLUGIN_DIR ${paraview_lib_location}/../lib)
else()
  set(PYTHONQTPLUGIN_DIR ${paraview_lib_location}/paraview-${PARAVIEW_VERSION_MAJOR}.${PARAVIEW_VERSION_MINOR}/plugins/PythonQtPlugin)
endif()

find_library(PYTHONQTPLUGIN_LIBRARY PythonQtPlugin HINTS ${PYTHONQTPLUGIN_DIR}  DOC "ParaView PythonQt plugin library")
mark_as_advanced(PYTHONQTPLUGIN_LIBRARY)

if(WIN32 AND MSVC)
  if(MSVC_VERSION GREATER 1699)
    add_definitions(/D_ALLOW_KEYWORD_MACROS)
  endif()
  add_definitions(/Dinline=__inline)
endif()


if (APPLE)
  set(CMAKE_INSTALL_NAME_DIR "@executable_path/../Libraries")

  # ensure that we don't build forwarding executables on apple.
  set(VTK_BUILD_FORWARDING_EXECUTABLES FALSE)
endif()

add_subdirectory(Plugins)
add_subdirectory(LVCore)
add_subdirectory(Application)

# TODO: once we use Qt built from source on Windows, remove this manual installation
# currently: PythonQt fails to build against our own build of Qt
if (WIN32)
  install(FILES "${paraview_lib_location}/plugins/PythonQtPlugin/PythonQtPlugin.dll"
          DESTINATION "bin")

  # install some python libs needed to launch Lidarview from the install dir
  get_filename_component(python_lib_location ${Python3_LIBRARY} PATH)
  install(DIRECTORY "${python_lib_location}/../Lib" 
    DESTINATION "bin")

  install(FILES "${python_lib_location}/../python${Python3_VERSION_MAJOR}${Python3_VERSION_MINOR}.dll" 
    DESTINATION "bin")

  foreach (qt5_opengl_lib IN ITEMS opengl32sw libEGL libGLESv2 libEGLd
      Qt5Core Qt5Gui Qt5Widgets Qt5Help Qt5Network Qt5PrintSupport Qt5Sql)
    install(FILES "${Qt5_DIR}/../../../bin/${qt5_opengl_lib}.dll"
      DESTINATION "bin")
  endforeach ()
  install(DIRECTORY "${Qt5_DIR}/../../../plugins/platforms"
    DESTINATION "bin"
  )
  # see comment in: Superbuild/Projects/win32/lidarview.bundle.cmake
  install(DIRECTORY "${Qt5_DIR}/../../../plugins/styles"
    DESTINATION "bin"
  )
endif ()
