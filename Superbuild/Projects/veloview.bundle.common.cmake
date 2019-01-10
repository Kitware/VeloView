# Enable CPack packaging.
set(VeloViewSuperBuild_SOURCE_DIR "${CMAKE_CURRENT_LIST_DIR}/../")
include(${VeloViewSuperBuild_SOURCE_DIR}/../SoftwareInformation/branding.cmake)

# Include CMake scripts for geting the version from Git
include(Git)
include(ParaViewDetermineVersion)
# Sets VV_VERSION_{MAJOR,MINOR,PATCH} for git
set(VV_VERSION_FILE ${VeloViewSuperBuild_SOURCE_DIR}/../version.txt)
file(STRINGS "${VV_VERSION_FILE}" version_txt)
extract_version_components("${version_txt}" "VV")
determine_version(${VeloViewSuperBuild_SOURCE_DIR} ${GIT_EXECUTABLE} "VV")
# Update the hard-coded version
extract_version_components("${version_txt}" "VV_file")
if((version_txt VERSION_LESS VV_VERSION_FULL)
   OR (version_txt VERSION_EQUAL VV_VERSION_FULL
       AND (VV_file_VERSION_PATCH_EXTRA STRLESS VV_VERSION_PATCH_EXTRA)))
  message(STATUS "Outdated version file updated from ${version_txt} to ${VV_VERSION_FULL} in " ${VV_VERSION_FILE})
  file(WRITE "${VV_VERSION_FILE}" "${VV_VERSION_FULL}")
endif()
if(NOT (version_txt STREQUAL VV_VERSION_FULL))
  message(STATUS "Git version (${VV_VERSION_FULL}) differs from version in file (${version_txt}) at " ${VV_VERSION_FILE})
endif()

# Sets GD_YEAR, GD_MONTH, GD_DAY
include(${VeloViewSuperBuild_SOURCE_DIR}/Projects/getdate.cmake)
GET_DATE()
set(PACKAGE_TIMESTAMP "${GD_YEAR}${GD_MONTH}${GD_DAY}")

set(CPACK_COMPONENT_VELOVIEW_DISPLAY_NAME ${SOFTWARE_NAME})
set(CPACK_PACKAGE_VERSION_MAJOR ${VV_VERSION_MAJOR})
set(CPACK_PACKAGE_VERSION_MINOR ${VV_VERSION_MINOR})
set(CPACK_PACKAGE_VERSION_PATCH ${VV_VERSION_PATCH})
if (NOT VV_VERSION_IS_RELEASE)
  set(CPACK_PACKAGE_VERSION_PATCH ${VV_VERSION_PATCH}-${VV_VERSION_PATCH_EXTRA})
else()
endif()

if (NOT VV_VERSION_IS_RELEASE)
  set(CPACK_PACKAGE_FILE_NAME
      "${CPACK_PACKAGE_NAME}-${VV_VERSION_FULL}-${PACKAGE_TIMESTAMP}-${package_suffix}")
else()
  set(CPACK_PACKAGE_FILE_NAME
      "${CPACK_PACKAGE_NAME}-${VV_VERSION_FULL}-${package_suffix}")
endif()
message(STATUS "Bundled package name will be: ${CPACK_PACKAGE_FILE_NAME}" )

# Set the license file.
set(CPACK_RESOURCE_FILE_LICENSE "${VeloViewSuperBuild_SOURCE_DIR}/LICENSE")

set(veloview_executables
	${SOFTWARE_NAME})


if (qt5_enabled)
  include(qt5.functions)

  set(qt5_plugin_prefix)
  if (NOT WIN32)
    set(qt5_plugin_prefix "lib")
  endif ()

  set(qt5_plugins
    sqldrivers/${qt5_plugin_prefix}qsqlite)

  if (WIN32)
    list(APPEND qt5_plugins
      platforms/qwindows)
  elseif (APPLE)
    list(APPEND qt5_plugins
      platforms/libqcocoa
      printsupport/libcocoaprintersupport)
  elseif (UNIX)
    list(APPEND qt5_plugins
      platforms/libqxcb
      platforminputcontexts/libcomposeplatforminputcontextplugin
      xcbglintegrations/libqxcb-glx-integration)
  endif ()

  superbuild_install_qt5_plugin_paths(qt5_plugin_paths ${qt5_plugins})
else ()
  set(qt5_plugin_paths)
endif ()

