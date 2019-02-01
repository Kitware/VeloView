cmake_minimum_required(VERSION 2.8.8)

project(VeloViewSuperBuild)
set (SuperBuild_CMAKE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/CMake")
set (SuperBuild_PROJECTS_DIR "${CMAKE_CURRENT_SOURCE_DIR}/Projects")
set (SuperBuild_BINARY_DIR ${VeloViewSuperBuild_BINARY_DIR}) # required to do FindPackage(ParaView) inside Veloview's CMakeLists

set(qt_version 5)

macro (superbuild_setup_variables)
  include(SuperbuildVersionMacros)
  # if (paraview_SOURCE_SELECTION STREQUAL "git")
  #   # Assuming master; just use the latest version, but let the user set their
  #   # own version in case it is actually a branch from some other version.
  #   set(PARAVIEW_VERSION_DEFAULT "5.5.0"
  #     CACHE STRING "The default version of ParaView to use if it cannot be detected")
  #   mark_as_advanced(PARAVIEW_VERSION_DEFAULT)
  #   set(paraview_default_version "${PARAVIEW_VERSION_DEFAULT}")
  # elseif (paraview_SOURCE_SELECTION STREQUAL "source")
  #   # If it is a git repo, we'll get it that way, otherwise we will look at the
  #   # `version.txt` in the checkout.
  #   set(paraview_default_version "")
  # else ()
  #   # The selection is the version number; use it.
  #   set(paraview_default_version "${paraview_SOURCE_SELECTION}")
  # endif()
  # superbuild_set_version_variables(paraview "${paraview_default_version}" "paraview-version.cmake" "version.txt")
  # set(paraview_version "${paraview_version_major}.${paraview_version_minor}")
  set(paraview_version "5.4")
endmacro ()

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
    bzip2
    cxx11
    eigen
    liblas
    freetype
    paraview
    pcap
    png
    python
    pythonqt
    qt5
    veloview
    zlib
    ceres
    glog
    pcl
    qhull
    flann
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
  if ( (NOT "${CMAKE_GENERATOR}" MATCHES "^NMake.*$") OR
       (NOT "${CMAKE_GENERATOR}" MATCHES "^Visual Studio.*$"))
    # not use VS environment. We need to be pointed to nmake and devenv paths
    # since they are needed when building tools (qt, python, etc.)
    find_program(DEVENV_PATH devenv)
    find_program(NMAKE_PATH nmake)
    mark_as_advanced(DEVENV_PATH NMAKE_PATH)
  endif()
endif()

if(UNIX AND NOT DEFINED USE_SYSTEM_python)
   # it is still possible to pass -DUSE_SYSTEM_python=False to cmake
   message(STATUS "Defaulting USE_SYSTEM_python to True (was undefined and we are on UNIX)")
   set(USE_SYSTEM_python True CACHE BOOL INTERNAL)
endif()

function (superbuild_add_packaging)
  if (WIN32)
    set(generators
      ZIP
      NSIS)
  elseif (APPLE)
    set(generators
      DragNDrop)
  else ()
    set(generators
      TGZ
      TXZ)
  endif ()
  list(GET generators 0 default_generator)

  if (USE_SYSTEM_qt5)
    list(APPEND superbuild_export_variables
      Qt5_DIR)
  endif ()

  # pass some variables that are required for packaging step:
  list(APPEND superbuild_export_variables
    PARAVIEW_VERSION
    package_suffix
    )

  set(default_package veloview)
  foreach (generator IN LISTS generators)
      superbuild_add_extra_package_test(veloview "${generator}"
      LABELS  "VeloView" # does not have to be ${SOFTWARE_NAME} I think
      TIMEOUT 6400)
  endforeach ()

  if (default_package)
    superbuild_enable_install_target("${default_package}/${default_generator}")
  endif ()
endfunction ()

function (superbuild_add_tests)
  # TODO: add existing tests
  # add_subdirectory("${CMAKE_SOURCE_DIR}/tests" "${CMAKE_BINARY_DIR}/tests")
endfunction ()

# skiped module as defined in paraview superbuild.
# One difference: pythonQt needs Multimedia so we don't skip it.
set(qt5_skip_modules
  qtspeech
  qtconnectivity
  qtgamepad
  qtlocation
  qtsensors
  qtserialport
  qtwayland
  qtwebchannel
  qtwebengine
  qtwebsockets)
list(APPEND qt5_process_environment
  BUILD_IN_SOURCE 1)

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
