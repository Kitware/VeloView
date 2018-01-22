cmake_minimum_required(VERSION 2.8.8)

project(VeloViewSuperBuild)
set (SuperBuild_CMAKE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/CMake")
set (SuperBuild_PROJECTS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Projects")
set (SuperBuild_BINARY_DIR ${VeloViewSuperBuild_BINARY_DIR})

#------------------------------------------------------------------------------
# Set a default build type if none was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'RelWithDebInfo' as none was specified.")
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release"
    "MinSizeRel" "RelWithDebInfo")
endif()

#-----------------------------------------------------------------------------
# Collect information about the build platform.
include(CheckTypeSize)
check_type_size(void* VOID_PTR_SIZE BUILTIN_TYPES_ONLY)
if (VOID_PTR_SIZE EQUAL 8)
  set(VV_BUILD_ARCHITECTURE "64")
else ()
  set(VV_BUILD_ARCHITECTURE "32")
endif()

# Set suffix to be used for generating archives. This ensures that the package
# files have decent names that we can directly upload to the website.
set (package_suffix "${CMAKE_SYSTEM_NAME}-${VV_BUILD_ARCHITECTURE}bit")

#-----------------------------------------------------------------------------
# Setup CMAKE_MODULE_PATH so that platform specific configurations are processed
# before the generic ones.
set (CMAKE_MODULE_PATH
  "${SuperBuild_PROJECTS_DIR}"
  "${SuperBuild_CMAKE_DIR}"
  "${CMAKE_CURRENT_SOURCE_DIR}"
  "${CMAKE_CURRENT_SOURCE_DIR}/../CMake"
  ${CMAKE_MODULE_PATH})

function (superbuild_find_projects var)
  # Some of these allow using system libraries.
  set(projects
    boost
    cxx11
    eigen
    liblas
    freetype
    paraview
    pcap
    png
    python
    pythonqt
    qt
    qt4
    qt5
    veloview
    veloview.bundle
    zlib
    )

  if (UNIX)
    list(APPEND projects
      libxml2
    )

    if (NOT APPLE)
      list(APPEND projects
        fontconfig
        gperf
      )
    endif()
  endif()

  set("${var}"
    ${projects}
    PARENT_SCOPE)
endfunction ()


if (WIN32)
  # Since we only support superbuild compilation with ninja, let's provide it
  file(COPY ${SuperBuild_PROJECTS_DIR}/win32/ninja.exe
       DESTINATION ${CMAKE_CURRENT_BINARY_DIR})
  if ( (NOT "${CMAKE_GENERATOR}" MATCHES "^NMake.*$") OR
       (NOT "${CMAKE_GENERATOR}" MATCHES "^Visual Studio.*$"))
    # not use VS environment. We need to be pointed to nmake and devenv paths
    # since they are needed when building tools (qt, python, etc.)
    find_program(DEVENV_PATH devenv)
    find_program(NMAKE_PATH nmake)
    mark_as_advanced(DEVENV_PATH NMAKE_PATH)
  endif()
endif()

set(boost_libraries
  chrono
  date_time
  filesystem
  iostreams
  program_options
  system
  regex
  thread)
set(boost_extra_options
  "-sNO_BZIP2=1")

list(APPEND superbuild_version_files
  "${CMAKE_CURRENT_LIST_DIR}/versions.cmake")
list(APPEND superbuild_project_roots
  "${SuperBuild_PROJECTS_DIR}")

set(_superbuild_default_cxx11 ON)

add_subdirectory(common-superbuild)
